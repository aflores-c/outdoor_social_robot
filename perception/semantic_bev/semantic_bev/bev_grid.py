"""
Multi-layer Bird's Eye View grid with global accumulation.

Coordinate conventions
──────────────────────
Odom / map frame  : x = forward, y = left, z = up  (ROS standard)
Grid cell (row, col):
  • col increases with x  →  col = 0 is at min-x (robot's back / right boundary)
  • row increases with y  →  row = 0 is at min-y (robot's right boundary)
  • This matches nav_msgs/OccupancyGrid layout  (row-major, origin at bottom-left)

Debug image (flipped for display):
  • np.flipud applied so robot forward (x+) appears at the top of the image.
"""

import numpy as np
import cv2

from .semantic_classes import (
    SemanticClass, CLASS_COLORS_BGR, NUM_CLASSES,
    merge_grids, class_grid_to_cost,
)


class BEVGrid:
    # Global map covers ±(GLOBAL_HALF_M) from the odom origin in each axis.
    # At 0.1 m/cell → 3000×3000 cells = 9 MB per layer — well within budget.
    GLOBAL_HALF_M: float = 150.0

    def __init__(self, local_width_m: float, local_height_m: float,
                 resolution: float) -> None:
        self.res   = float(resolution)
        self.lw    = float(local_width_m)   # local map x-extent  (m)
        self.lh    = float(local_height_m)  # local map y-extent  (m)
        self.lcols = int(round(local_width_m  / resolution))  # x cells
        self.lrows = int(round(local_height_m / resolution))  # y cells

        half_m = self.GLOBAL_HALF_M
        self.gcells = int(round(2 * half_m / resolution))  # global side length
        self.ghalf  = self.gcells // 2                     # index of odom origin

        # ── Global accumulated layers (persist as robot moves) ────────────────
        # Layout: [row, col] → row 0 = min-y, col 0 = min-x
        self.global_static   = np.zeros((self.gcells, self.gcells), dtype=np.int8)
        self.global_semantic = np.zeros((self.gcells, self.gcells), dtype=np.int8)

        # ── Local transient layers (rebuilt every update cycle) ───────────────
        self._dyn   = np.zeros((self.lrows, self.lcols), dtype=np.int8)
        self._fused = np.zeros((self.lrows, self.lcols), dtype=np.int8)

        self.robot_x = 0.0
        self.robot_y = 0.0

    # ── Coordinate helpers ────────────────────────────────────────────────────

    def _xy_to_global_rc(self, x: np.ndarray,
                          y: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
        """Odom (x, y) → global grid (row, col); row increases with y."""
        col = (self.ghalf + x / self.res).astype(np.int32)
        row = (self.ghalf + y / self.res).astype(np.int32)
        return row, col

    def _valid_global(self, row: np.ndarray,
                       col: np.ndarray) -> np.ndarray:
        """Boolean mask of indices inside the global array bounds."""
        return ((row >= 0) & (row < self.gcells) &
                (col >= 0) & (col < self.gcells))

    def _local_origin(self) -> tuple[float, float]:
        """Bottom-left corner of the local map in odom frame (m)."""
        return self.robot_x - self.lw / 2.0, self.robot_y - self.lh / 2.0

    def _global_slice_of_local(self) -> tuple[slice, slice]:
        """
        Return numpy slices into the global array that correspond to the
        current local window (clamped to global bounds).
        Returns (row_slice, col_slice) for global array,
        and (lr_slice, lc_slice) for local array.
        """
        lox, loy = self._local_origin()
        gr0 = int(self.ghalf + loy / self.res)
        gc0 = int(self.ghalf + lox / self.res)

        # Clamp global indices
        gr0c = max(0, gr0);  gr1c = min(self.gcells, gr0 + self.lrows)
        gc0c = max(0, gc0);  gc1c = min(self.gcells, gc0 + self.lcols)

        # Corresponding local indices
        lr0 = gr0c - gr0;  lr1 = lr0 + (gr1c - gr0c)
        lc0 = gc0c - gc0;  lc1 = lc0 + (gc1c - gc0c)

        return (
            (slice(gr0c, gr1c), slice(gc0c, gc1c)),
            (slice(lr0,  lr1),  slice(lc0,  lc1)),
        )

    # ── Layer update API ──────────────────────────────────────────────────────

    def update_robot_pose(self, robot_x: float, robot_y: float) -> None:
        self.robot_x = robot_x
        self.robot_y = robot_y

    def insert_static_points(self, x: np.ndarray, y: np.ndarray,
                              class_ids: np.ndarray | None = None,
                              default_class: int = SemanticClass.OBSTACLE) -> None:
        """
        Accumulate odom-frame obstacle/geometry points into the global static layer.
        class_ids: per-point class override; if None, default_class is used for all.
        """
        if len(x) == 0:
            return
        row, col = self._xy_to_global_rc(x, y)
        mask = self._valid_global(row, col)
        row, col = row[mask], col[mask]
        ids = class_ids[mask] if class_ids is not None else np.full(len(row), default_class, dtype=np.int8)

        # Vectorised priority merge via numpy advanced indexing
        existing = self.global_static[row, col]
        overlay  = ids.astype(np.int8)
        self.global_static[row, col] = merge_grids(existing, overlay)

    def insert_semantic_points(self, x: np.ndarray, y: np.ndarray,
                                class_ids: np.ndarray) -> None:
        """Accumulate camera-derived semantic labels into the global semantic layer."""
        if len(x) == 0:
            return
        row, col = self._xy_to_global_rc(x, y)
        mask = self._valid_global(row, col)
        row, col = row[mask], col[mask]
        ids = class_ids[mask].astype(np.int8)

        existing = self.global_semantic[row, col]
        self.global_semantic[row, col] = merge_grids(existing, ids)

    def update_dynamic_layer(self, x: np.ndarray, y: np.ndarray,
                              class_id: int = SemanticClass.OBSTACLE) -> None:
        """
        Rebuild the dynamic obstacle layer from the current LiDAR scan.
        Called once per scan; previous dynamic data is discarded.
        """
        self._dyn[:] = int(SemanticClass.UNKNOWN)
        if len(x) == 0:
            return
        lox, loy = self._local_origin()
        col = ((x - lox) / self.res).astype(np.int32)
        row = ((y - loy) / self.res).astype(np.int32)
        valid = ((row >= 0) & (row < self.lrows) &
                 (col >= 0) & (col < self.lcols))
        self._dyn[row[valid], col[valid]] = class_id

    # ── Fusion ────────────────────────────────────────────────────────────────

    def fuse_layers(self) -> np.ndarray:
        """
        Merge all layers with priority rules and return the fused class-ID grid.
        Priority (highest → lowest):
          dynamic obstacles (current scan)
          > global static geometry (FAST-LIO map)
          > global semantic labels (camera / YOLO)
        """
        g_slice, l_slice = self._global_slice_of_local()

        fused = np.zeros((self.lrows, self.lcols), dtype=np.int8)

        # 1. Lay down accumulated semantic surface labels (lowest seniority)
        sem_win = self.global_semantic[g_slice[0], g_slice[1]]
        fused[l_slice[0], l_slice[1]] = merge_grids(
            fused[l_slice[0], l_slice[1]], sem_win
        )

        # 2. Overlay static geometry (FAST-LIO registered map)
        sta_win = self.global_static[g_slice[0], g_slice[1]]
        fused[l_slice[0], l_slice[1]] = merge_grids(
            fused[l_slice[0], l_slice[1]], sta_win
        )

        # 3. Dynamic obstacles from current scan always win
        dyn_obs = self._dyn != int(SemanticClass.UNKNOWN)
        fused[dyn_obs] = self._dyn[dyn_obs]

        # 4. Always mark the robot's immediate footprint as free
        cr, cc = self.lrows // 2, self.lcols // 2
        fused[cr - 1:cr + 2, cc - 1:cc + 2] = int(SemanticClass.FREE)

        self._fused = fused
        return fused

    # ── Output helpers ────────────────────────────────────────────────────────

    def get_cost_grid(self) -> np.ndarray:
        """Return OccupancyGrid data (int8): -1=unknown, 0=free, 1–100=occupied."""
        return class_grid_to_cost(self._fused)

    def get_debug_image(self) -> np.ndarray:
        """
        Return a BGR uint8 image (H, W, 3) for visualization.
        Image is flipped so robot forward (x+) appears at the top.
        """
        img = np.zeros((self.lrows, self.lcols, 3), dtype=np.uint8)
        for cls_id, color in CLASS_COLORS_BGR.items():
            img[self._fused == cls_id] = color
        # White cross at robot center
        cr, cc = self.lrows // 2, self.lcols // 2
        cv2.line(img, (cc - 8, cr), (cc + 8, cr), (255, 255, 255), 1)
        cv2.line(img, (cc, cr - 8), (cc, cr + 8), (255, 255, 255), 1)
        # Flip so robot forward (x+) is at the top of the image
        return np.flipud(img)

    def get_obstacle_points_odom(self) -> np.ndarray:
        """Return (N, 3) float32 obstacle cell centres in odom frame, z=0."""
        obstacle_classes = (
            int(SemanticClass.OBSTACLE),
            int(SemanticClass.CURB),
            int(SemanticClass.PEDESTRIAN),
            int(SemanticClass.VEHICLE),
            int(SemanticClass.BICYCLE),
        )
        mask = np.isin(self._fused, obstacle_classes)
        rows, cols = np.where(mask)
        if len(rows) == 0:
            return np.zeros((0, 3), dtype=np.float32)
        lox, loy = self._local_origin()
        x = (lox + (cols + 0.5) * self.res).astype(np.float32)
        y = (loy + (rows + 0.5) * self.res).astype(np.float32)
        return np.column_stack([x, y, np.zeros(len(x), dtype=np.float32)])

    def get_semantic_points_odom(self) -> tuple[np.ndarray, np.ndarray]:
        """Return ((N,3) xyz, (N,) class_id) for all non-unknown cells in odom frame."""
        rows, cols = np.where(self._fused > int(SemanticClass.UNKNOWN))
        if len(rows) == 0:
            return np.zeros((0, 3), dtype=np.float32), np.zeros(0, dtype=np.int8)
        lox, loy = self._local_origin()
        x = (lox + (cols + 0.5) * self.res).astype(np.float32)
        y = (loy + (rows + 0.5) * self.res).astype(np.float32)
        pts = np.column_stack([x, y, np.zeros(len(x), dtype=np.float32)])
        return pts, self._fused[rows, cols]
