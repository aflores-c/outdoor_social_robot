"""
semantic_bev_node.py — Local semantic Bird's Eye View map node.

Sensor roles
────────────
  Velodyne VLP-32  →  3D geometry for BOTH obstacle detection AND depth of
                       semantic labels (projects LiDAR points onto camera image)
  Intel D455 RGB   →  Only used for YOLO segmentation mask (pixel labels)
  Intel D455 depth →  Not used (6 m limit is insufficient outdoors; LiDAR takes over)
  FAST-LIO odom    →  Robot pose + registered map cloud for static geometry layer

LiDAR-camera fusion (the key step)
────────────────────────────────────
  For each LiDAR point P in velodyne frame:
    1. Transform P → camera frame using TF (velodyne → camera_link)
    2. Project P onto image plane with camera intrinsics → pixel (u, v)
    3. Sample YOLO segmentation mask at (u, v) → semantic class
    4. Transform P → odom frame using TF (velodyne → odom)
    5. Insert the real 3D LiDAR position into the BEV semantic layer
       with the camera-derived class label

  Result: semantic labels from the camera applied at LiDAR range (100 m).

Output topics
─────────────
  /semantic_bev/grid            nav_msgs/OccupancyGrid
  /semantic_bev/classes         visualization_msgs/MarkerArray
  /semantic_bev/debug_image     sensor_msgs/Image
  /semantic_bev/obstacle_cloud  sensor_msgs/PointCloud2
  /semantic_bev/semantic_cloud  sensor_msgs/PointCloud2
"""

from __future__ import annotations

import threading
from typing import Optional

import cv2
import numpy as np
from scipy.spatial.transform import Rotation

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from cv_bridge import CvBridge

from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import Point, TransformStamped, Vector3
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

from tf2_ros import (Buffer, TransformListener,
                     LookupException, ConnectivityException,
                     ExtrapolationException)

from .bev_grid import BEVGrid
from .semantic_classes import SemanticClass, CLASS_COLORS_BGR, CLASS_NAMES

# ── PointCloud2 helpers ───────────────────────────────────────────────────────

_DTYPE_MAP = {
    1: ('int8', 1), 2: ('uint8', 1), 3: ('int16', 2), 4: ('uint16', 2),
    5: ('int32', 4), 6: ('uint32', 4), 7: ('float32', 4), 8: ('float64', 8),
}


def _cloud_to_xyz(msg: PointCloud2) -> np.ndarray:
    """Parse a PointCloud2 to (N, 3) float32. Uses structured numpy dtype — no Python loops."""
    field_map = {f.name: f for f in msg.fields}
    if not all(k in field_map for k in ('x', 'y', 'z')):
        return np.zeros((0, 3), dtype=np.float32)
    step = msg.point_step
    dtype_list: list = []
    cur = 0
    for f in sorted(msg.fields, key=lambda x: x.offset):
        if f.offset > cur:
            dtype_list.append((f'_p{cur}', f'V{f.offset - cur}'))
        np_type, size = _DTYPE_MAP.get(f.datatype, ('float32', 4))
        dtype_list.append((f.name, np_type))
        cur = f.offset + size
    if cur < step:
        dtype_list.append(('_pe', f'V{step - cur}'))
    raw = np.frombuffer(bytes(msg.data), dtype=np.dtype(dtype_list))
    pts = np.column_stack([raw['x'].astype(np.float32),
                           raw['y'].astype(np.float32),
                           raw['z'].astype(np.float32)])
    return pts[np.isfinite(pts).all(axis=1)]


def _xyz_to_cloud(pts: np.ndarray, frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    msg.height = 1;  msg.width = len(pts)
    msg.is_dense = True;  msg.is_bigendian = False
    msg.point_step = 12;  msg.row_step = 12 * msg.width
    msg.fields = [
        PointField(name='x', offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y', offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z', offset=8,  datatype=PointField.FLOAT32, count=1),
    ]
    msg.data = pts.astype(np.float32).tobytes()
    return msg


def _xyzc_to_cloud(pts: np.ndarray, classes: np.ndarray,
                    frame_id: str, stamp) -> PointCloud2:
    msg = PointCloud2()
    msg.header.frame_id = frame_id
    msg.header.stamp = stamp
    n = len(pts)
    msg.height = 1;  msg.width = n
    msg.is_dense = True;  msg.is_bigendian = False
    msg.point_step = 16;  msg.row_step = 16 * n
    msg.fields = [
        PointField(name='x',        offset=0,  datatype=PointField.FLOAT32, count=1),
        PointField(name='y',        offset=4,  datatype=PointField.FLOAT32, count=1),
        PointField(name='z',        offset=8,  datatype=PointField.FLOAT32, count=1),
        PointField(name='class_id', offset=12, datatype=PointField.FLOAT32, count=1),
    ]
    data = np.column_stack([pts, classes.astype(np.float32)])
    msg.data = data.astype(np.float32).tobytes()
    return msg


# ── Main node ────────────────────────────────────────────────────────────────

class SemanticBEVNode(Node):

    def __init__(self) -> None:
        super().__init__('semantic_bev_node')
        self._declare_params()
        p = self._p

        self._grid = BEVGrid(
            local_width_m=p('map_width_m'),
            local_height_m=p('map_height_m'),
            resolution=p('resolution'),
        )

        self._tf_buf = Buffer()
        self._tf_listener = TransformListener(self._tf_buf, self)
        self._bridge = CvBridge()

        # ── Caches ───────────────────────────────────────────────────────────
        self._lock = threading.Lock()
        self._lidar_cloud:   Optional[PointCloud2] = None
        self._seg_mask:      Optional[np.ndarray]  = None  # uint8 class-ID image
        self._seg_stamp:     Optional[Time]         = None
        self._cam_K:         Optional[np.ndarray]  = None  # (3, 3) intrinsic matrix
        self._cam_frame:     str = p('camera_frame')
        self._yolo_lut:      Optional[np.ndarray]  = None  # class-ID remapping LUT

        # ── QoS ──────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability=ReliabilityPolicy.BEST_EFFORT,
            durability=DurabilityPolicy.VOLATILE,
            history=HistoryPolicy.KEEP_LAST, depth=1)
        reliable_qos = QoSProfile(
            reliability=ReliabilityPolicy.RELIABLE, depth=5)

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(PointCloud2, p('lidar_topic'),
                                 self._on_lidar, sensor_qos)
        self.create_subscription(PointCloud2, p('fastlio_cloud_topic'),
                                 self._on_fastlio_cloud, sensor_qos)
        self.create_subscription(Image, p('seg_mask_topic'),
                                 self._on_seg_mask, sensor_qos)
        self.create_subscription(CameraInfo, p('cam_info_topic'),
                                 self._on_cam_info, reliable_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_grid    = self.create_publisher(OccupancyGrid,  '/semantic_bev/grid',          10)
        self._pub_markers = self.create_publisher(MarkerArray,    '/semantic_bev/classes',        10)
        self._pub_debug   = self.create_publisher(Image,          '/semantic_bev/debug_image',    10)
        self._pub_obs_pc  = self.create_publisher(PointCloud2,    '/semantic_bev/obstacle_cloud', sensor_qos)
        self._pub_sem_pc  = self.create_publisher(PointCloud2,    '/semantic_bev/semantic_cloud', sensor_qos)

        self.create_timer(1.0 / p('update_rate'), self._update)

        self.get_logger().info(
            f'SemanticBEVNode ready | '
            f'{p("map_width_m")}×{p("map_height_m")} m @ {p("resolution")} m/cell | '
            f'{p("update_rate")} Hz | '
            f'semantic depth: LiDAR ({p("lidar_topic")})'
        )

    # ── Parameters ────────────────────────────────────────────────────────────

    def _declare_params(self) -> None:
        d = self.declare_parameter
        d('odom_frame',          'camera_init')
        d('base_frame',          'base_link')
        d('camera_frame',        'camera_link')
        d('lidar_frame',         'velodyne')
        d('lidar_topic',         '/velodyne_points')
        d('fastlio_cloud_topic', '/cloud_registered')
        d('seg_mask_topic',      '/yolo/seg_mask')
        d('cam_info_topic',      '/camera/color/camera_info')
        d('resolution',          0.1)
        d('map_width_m',         40.0)
        d('map_height_m',        40.0)
        d('update_rate',         5.0)
        d('ground_z_max',        0.10)   # points at or below this → ground, skip
        d('obstacle_z_max',      2.50)   # points above this → skip
        d('lidar_max_range_m',   80.0)   # ignore LiDAR returns beyond this
        d('seg_time_tol_s',      0.5)    # max age diff between lidar and seg_mask
        d('use_yolo_class_map',  True)
        d('yolo_class_map', [
            0, 7,   # person      → PEDESTRIAN
            1, 9,   # bicycle     → BICYCLE
            2, 8,   # car         → VEHICLE
            3, 9,   # motorcycle  → BICYCLE
            5, 8,   # bus         → VEHICLE
            7, 8,   # truck       → VEHICLE
        ])

    def _p(self, name: str):
        return self.get_parameter(name).value

    # ── Callbacks ─────────────────────────────────────────────────────────────

    def _on_lidar(self, msg: PointCloud2) -> None:
        with self._lock:
            self._lidar_cloud = msg

    def _on_fastlio_cloud(self, msg: PointCloud2) -> None:
        """Accumulate FAST-LIO registered cloud into the global static geometry layer."""
        pts = _cloud_to_xyz(msg)
        if len(pts) == 0:
            return
        odom_frame = self._p('odom_frame')
        src = msg.header.frame_id or odom_frame
        if src != odom_frame:
            T = self._get_transform(odom_frame, src, Time.from_msg(msg.header.stamp))
            if T is None:
                return
            pts = self._apply_transform(pts, T)
        g_max = self._p('ground_z_max')
        obs_max = self._p('obstacle_z_max')
        keep = (pts[:, 2] > g_max) & (pts[:, 2] < obs_max)
        pts = pts[keep]
        if len(pts) == 0:
            return
        with self._lock:
            self._grid.insert_static_points(pts[:, 0], pts[:, 1])

    def _on_cam_info(self, msg: CameraInfo) -> None:
        """Store camera intrinsics once."""
        with self._lock:
            if self._cam_K is None:
                self._cam_K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
                self._cam_frame = msg.header.frame_id or self._p('camera_frame')
                self.get_logger().info(
                    f'Camera intrinsics stored | '
                    f'fx={self._cam_K[0,0]:.1f} fy={self._cam_K[1,1]:.1f} '
                    f'cx={self._cam_K[0,2]:.1f} cy={self._cam_K[1,2]:.1f} | '
                    f'frame: {self._cam_frame}'
                )

    def _on_seg_mask(self, msg: Image) -> None:
        """
        Segmentation mask where each pixel value = class ID (uint8 / mono8).

        YOLO integration
        ─────────────────
        Publish a mono8 Image on seg_mask_topic where pixel value is your
        YOLO model's class ID. Set use_yolo_class_map: true (default) to
        remap those IDs to SemanticClass IDs using yolo_class_map in config.

        If you publish SemanticClass IDs directly, set use_yolo_class_map: false.

        Placeholder: if YOLO is not yet connected, simply don't publish on
        this topic — the node still runs with LiDAR geometry only.
        """
        try:
            mask = self._bridge.imgmsg_to_cv2(msg, desired_encoding='mono8')
        except Exception as e:
            self.get_logger().warn(f'seg_mask decode error: {e}',
                                   throttle_duration_sec=5.0)
            return

        if self._p('use_yolo_class_map'):
            lut = self._get_yolo_lut()
            mask = lut[mask]

        with self._lock:
            self._seg_mask = mask
            self._seg_stamp = self.get_clock().now()

    # ── LiDAR-camera semantic fusion ──────────────────────────────────────────

    def _lidar_color_semantics(self,
                                pts_lidar: np.ndarray,
                                T_cam_lidar: np.ndarray,
                                T_odom_lidar: np.ndarray,
                                seg_mask: np.ndarray,
                                cam_K: np.ndarray) -> None:
        """
        Core LiDAR-camera fusion step.

        For every LiDAR point:
          1. Transform to camera frame                   (velodyne → camera_link)
          2. Project onto image plane using intrinsics   → pixel (u, v)
          3. Sample YOLO segmentation mask at (u, v)     → semantic class
          4. Transform original LiDAR point to odom      (velodyne → odom)
          5. Insert at true 3D LiDAR position into the global semantic layer

        This gives camera semantics at full LiDAR range (up to 80+ m),
        completely replacing the depth-image back-projection approach.
        """
        n = len(pts_lidar)
        if n == 0:
            return

        # ── Step 1: LiDAR → camera frame ─────────────────────────────────────
        p4 = np.hstack([pts_lidar, np.ones((n, 1), dtype=np.float32)])
        pts_cam = (T_cam_lidar.astype(np.float32) @ p4.T).T[:, :3]  # (N, 3)

        # Only points strictly in front of the camera lens (Z_cam > 0)
        front_mask = pts_cam[:, 2] > 0.1
        pts_cam    = pts_cam[front_mask]
        p4_front   = p4[front_mask]        # keep homogeneous for odom transform

        if len(pts_cam) == 0:
            return

        # ── Step 2: Pinhole projection → pixel (u, v) ────────────────────────
        fx = cam_K[0, 0];  fy = cam_K[1, 1]
        cx = cam_K[0, 2];  cy = cam_K[1, 2]
        Z  = pts_cam[:, 2]
        u  = (fx * pts_cam[:, 0] / Z + cx).astype(np.int32)
        v  = (fy * pts_cam[:, 1] / Z + cy).astype(np.int32)

        # ── Step 3: Discard projections outside image bounds ──────────────────
        H, W = seg_mask.shape[:2]
        in_fov = (u >= 0) & (u < W) & (v >= 0) & (v < H)
        u        = u[in_fov]
        v        = v[in_fov]
        p4_fov   = p4_front[in_fov]

        if len(u) == 0:
            return

        # ── Step 4: Sample mask → class label per LiDAR point ─────────────────
        labels = seg_mask[v, u]          # shape (N_fov,), dtype uint8
        labeled = labels > 0             # only semantically identified points
        if not np.any(labeled):
            return

        p4_labeled = p4_fov[labeled]
        cls        = labels[labeled].astype(np.int8)

        # ── Step 5: Transform labeled LiDAR points to odom frame ─────────────
        pts_odom = (T_odom_lidar.astype(np.float32) @ p4_labeled.T).T[:, :3]

        # Height filter: ignore ground and ceiling returns
        g_max   = self._p('ground_z_max')
        obs_max = self._p('obstacle_z_max')
        keep    = (pts_odom[:, 2] > g_max) & (pts_odom[:, 2] < obs_max)

        if not np.any(keep):
            return

        with self._lock:
            self._grid.insert_semantic_points(
                pts_odom[keep, 0], pts_odom[keep, 1], cls[keep]
            )

        self.get_logger().debug(
            f'Semantic LiDAR: {np.sum(labeled)} labeled pts '
            f'({np.sum(keep)} passed height filter)',
            throttle_duration_sec=1.0,
        )

    # ── Dynamic obstacle layer ────────────────────────────────────────────────

    def _update_dynamic_layer(self, pts_lidar: np.ndarray,
                               T_odom_lidar: np.ndarray) -> None:
        """Ground-filter the raw scan and write fresh obstacle cells."""
        if len(pts_lidar) == 0:
            return
        n  = len(pts_lidar)
        p4 = np.hstack([pts_lidar, np.ones((n, 1), dtype=np.float32)])
        pts_odom = (T_odom_lidar.astype(np.float32) @ p4.T).T[:, :3]

        g_max   = self._p('ground_z_max')
        obs_max = self._p('obstacle_z_max')
        keep    = (pts_odom[:, 2] > g_max) & (pts_odom[:, 2] < obs_max)

        with self._lock:
            self._grid.update_dynamic_layer(
                pts_odom[keep, 0], pts_odom[keep, 1]
            )

    # ── Main update loop ──────────────────────────────────────────────────────

    def _update(self) -> None:
        odom_frame  = self._p('odom_frame')
        base_frame  = self._p('base_frame')
        lidar_frame = self._p('lidar_frame')
        stamp       = self.get_clock().now()

        # ── Robot pose ────────────────────────────────────────────────────────
        T_odom_base = self._get_transform(odom_frame, base_frame, stamp)
        if T_odom_base is None:
            self.get_logger().warn(
                f'No TF {odom_frame}←{base_frame}; skipping',
                throttle_duration_sec=3.0)
            return

        self._grid.update_robot_pose(
            float(T_odom_base[0, 3]),
            float(T_odom_base[1, 3]),
        )

        # ── Grab cached data (snapshot, no lock held during processing) ───────
        with self._lock:
            lidar_msg  = self._lidar_cloud
            seg_mask   = self._seg_mask
            seg_stamp  = self._seg_stamp
            cam_K      = self._cam_K
            cam_frame  = self._cam_frame

        if lidar_msg is None:
            return

        lidar_stamp = Time.from_msg(lidar_msg.header.stamp)

        # ── Parse LiDAR cloud once — used for both paths below ────────────────
        pts_lidar = _cloud_to_xyz(lidar_msg)

        # Range filter: skip returns beyond max useful range
        max_r = self._p('lidar_max_range_m')
        r2    = pts_lidar[:, 0]**2 + pts_lidar[:, 1]**2 + pts_lidar[:, 2]**2
        pts_lidar = pts_lidar[r2 < max_r**2]

        # ── Transform: velodyne → odom (needed by both paths) ─────────────────
        T_odom_lidar = self._get_transform(odom_frame, lidar_frame, lidar_stamp)
        if T_odom_lidar is None:
            self.get_logger().warn(
                f'No TF {odom_frame}←{lidar_frame}; skipping',
                throttle_duration_sec=3.0)
            return

        # ── Path A: semantic LiDAR coloring ──────────────────────────────────
        # Requires: segmentation mask + camera intrinsics + TF velodyne→camera
        if seg_mask is not None and cam_K is not None:
            seg_age_s = abs((stamp - seg_stamp).nanoseconds) * 1e-9
            if seg_age_s <= self._p('seg_time_tol_s'):
                T_cam_lidar = self._get_transform(cam_frame, lidar_frame, lidar_stamp)
                if T_cam_lidar is not None:
                    self._lidar_color_semantics(
                        pts_lidar, T_cam_lidar, T_odom_lidar, seg_mask, cam_K
                    )

        # ── Path B: dynamic obstacle layer from raw scan ──────────────────────
        self._update_dynamic_layer(pts_lidar, T_odom_lidar)

        # ── Fuse all layers and publish ───────────────────────────────────────
        self._grid.fuse_layers()
        stamp_msg = stamp.to_msg()
        self._publish_grid(stamp_msg, odom_frame)
        self._publish_debug(stamp_msg)
        self._publish_obstacle_cloud(stamp_msg, odom_frame)
        self._publish_semantic_cloud(stamp_msg, odom_frame)
        self._publish_markers(stamp_msg, odom_frame)

    # ── Publishers ────────────────────────────────────────────────────────────

    def _publish_grid(self, stamp, frame_id: str) -> None:
        cost = self._grid.get_cost_grid()
        lox, loy = self._grid._local_origin()
        msg = OccupancyGrid()
        msg.header.stamp = stamp
        msg.header.frame_id = frame_id
        msg.info.resolution = self._grid.res
        msg.info.width  = self._grid.lcols
        msg.info.height = self._grid.lrows
        msg.info.origin.position.x = lox
        msg.info.origin.position.y = loy
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0
        msg.data = cost.flatten().tolist()
        self._pub_grid.publish(msg)

    def _publish_debug(self, stamp) -> None:
        img = self._grid.get_debug_image()
        msg = self._bridge.cv2_to_imgmsg(img, encoding='bgr8')
        msg.header.stamp = stamp
        self._pub_debug.publish(msg)

    def _publish_obstacle_cloud(self, stamp, frame_id: str) -> None:
        pts = self._grid.get_obstacle_points_odom()
        if len(pts) == 0:
            return
        self._pub_obs_pc.publish(_xyz_to_cloud(pts, frame_id, stamp))

    def _publish_semantic_cloud(self, stamp, frame_id: str) -> None:
        pts, cls = self._grid.get_semantic_points_odom()
        if len(pts) == 0:
            return
        self._pub_sem_pc.publish(_xyzc_to_cloud(pts, cls, frame_id, stamp))

    def _publish_markers(self, stamp, frame_id: str) -> None:
        fused = self._grid._fused
        ma    = MarkerArray()
        lox, loy = self._grid._local_origin()
        res  = self._grid.res
        lt   = DurationMsg(sec=2, nanosec=0)

        for cls_id, color_bgr in CLASS_COLORS_BGR.items():
            if cls_id in (int(SemanticClass.UNKNOWN), int(SemanticClass.FREE)):
                continue
            rows, cols = np.where(fused == cls_id)
            if len(rows) == 0:
                continue
            m = Marker()
            m.header.frame_id = frame_id
            m.header.stamp    = stamp
            m.ns     = CLASS_NAMES.get(cls_id, str(cls_id))
            m.id     = cls_id
            m.type   = Marker.CUBE_LIST
            m.action = Marker.ADD
            m.scale  = Vector3(x=res, y=res, z=0.15)
            m.color  = ColorRGBA(
                r=color_bgr[2] / 255.0,
                g=color_bgr[1] / 255.0,
                b=color_bgr[0] / 255.0,
                a=0.75)
            m.lifetime = lt
            x_vals = lox + (cols + 0.5) * res
            y_vals = loy + (rows + 0.5) * res
            m.points = [Point(x=float(x), y=float(y), z=0.05)
                        for x, y in zip(x_vals, y_vals)]
            ma.markers.append(m)

        existing_ids = {m.id for m in ma.markers}
        for cls_id in CLASS_NAMES:
            if cls_id not in existing_ids and cls_id not in (
                    int(SemanticClass.UNKNOWN), int(SemanticClass.FREE)):
                del_m = Marker()
                del_m.header.frame_id = frame_id
                del_m.header.stamp    = stamp
                del_m.ns     = CLASS_NAMES.get(cls_id, str(cls_id))
                del_m.id     = cls_id
                del_m.action = Marker.DELETE
                ma.markers.append(del_m)

        self._pub_markers.publish(ma)

    # ── TF / transform helpers ────────────────────────────────────────────────

    def _get_transform(self, target: str, source: str,
                        stamp: Time) -> Optional[np.ndarray]:
        """Return 4×4 float64 homogeneous transform or None if unavailable."""
        for t in (stamp, rclpy.time.Time()):   # try exact stamp, fall back to latest
            try:
                tf: TransformStamped = self._tf_buf.lookup_transform(
                    target, source, t, timeout=Duration(seconds=0.05))
                tr = tf.transform.translation
                q  = tf.transform.rotation
                R  = Rotation.from_quat([q.x, q.y, q.z, q.w]).as_matrix()
                T  = np.eye(4, dtype=np.float64)
                T[:3, :3] = R
                T[:3, 3]  = [tr.x, tr.y, tr.z]
                return T
            except (LookupException, ConnectivityException,
                    ExtrapolationException):
                continue
        return None

    @staticmethod
    def _apply_transform(pts: np.ndarray, T: np.ndarray) -> np.ndarray:
        n  = len(pts)
        p4 = np.hstack([pts, np.ones((n, 1), dtype=pts.dtype)])
        return (T @ p4.T).T[:, :3]

    # ── YOLO LUT ──────────────────────────────────────────────────────────────

    def _get_yolo_lut(self) -> np.ndarray:
        """Build and cache a 256-entry uint8 LUT: yolo_class_id → SemanticClass ID."""
        if self._yolo_lut is not None:
            return self._yolo_lut
        lut = np.zeros(256, dtype=np.uint8)
        raw = self._p('yolo_class_map')
        if len(raw) % 2 == 0:
            for i in range(0, len(raw), 2):
                yolo_id, sem_id = int(raw[i]), int(raw[i + 1])
                if 0 <= yolo_id < 256:
                    lut[yolo_id] = sem_id
        self._yolo_lut = lut
        return lut


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SemanticBEVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
