"""
segmentation_node.py
─────────────────────
ROS 2 node that runs semantic segmentation and publishes a class-ID mask
consumed by semantic_bev_node.

Two models run at independent, configurable rates:

  SegFormer  (slow, ~2–5 Hz)  ──→  surfaces: road, sidewalk, grass, curb
  YOLOv8-seg (fast, ~10 Hz)   ──→  objects:  person, car, bicycle, …

The masks are merged each YOLO cycle — YOLO objects override SegFormer
surface labels (objects have higher semantic priority).

Published topic
───────────────
  /yolo/seg_mask   sensor_msgs/Image  mono8
    pixel value = SemanticClass ID (matches semantic_bev semantic_classes.py)

Performance presets
───────────────────
  RTX desktop  : config/rtx_desktop.yaml  — larger models, FP32, high res
  Jetson Orin  : config/jetson_orin.yaml  — smaller models, FP16, lower res
                                            optionally TensorRT (.engine file)
"""

from __future__ import annotations

import threading
import time
from typing import Optional

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from .yolo_inferencer import YOLOInferencer
from .segformer_inferencer import SegFormerInferencer


class SegmentationNode(Node):

    def __init__(self) -> None:
        super().__init__('segmentation_node')
        self._declare_params()
        p = self._p

        self._bridge = CvBridge()
        self._lock   = threading.Lock()

        # Latest raw image from camera
        self._latest_bgr:   Optional[np.ndarray] = None
        self._latest_stamp  = None

        # Latest SegFormer result (updated at its own slower rate)
        self._surface_mask: Optional[np.ndarray] = None

        # ── Load models ───────────────────────────────────────────────────────
        # Models are loaded in the constructor (blocking).
        # On first run ultralytics and HuggingFace will download weights.
        device = p('device')

        self._yolo = YOLOInferencer(
            model_path      = p('yolo_model'),
            device          = device,
            conf_threshold  = p('yolo_conf'),
            class_map_pairs = self._parse_class_map(p('yolo_class_map')),
            half            = p('use_half'),
            img_size        = p('yolo_img_size'),
        )

        self._segformer: Optional[SegFormerInferencer] = None
        if p('use_segformer'):
            self._segformer = SegFormerInferencer(
                model_name   = p('segformer_model'),
                device       = device,
                half         = p('use_half'),
                infer_width  = p('segformer_width'),
                infer_height = p('segformer_height'),
            )

        # ── QoS ──────────────────────────────────────────────────────────────
        sensor_qos = QoSProfile(
            reliability = ReliabilityPolicy.BEST_EFFORT,
            durability  = DurabilityPolicy.VOLATILE,
            history     = HistoryPolicy.KEEP_LAST,
            depth       = 1,
        )

        # ── Subscribers ───────────────────────────────────────────────────────
        self.create_subscription(
            Image, p('image_topic'), self._on_image, sensor_qos)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_mask  = self.create_publisher(
            Image, '/yolo/seg_mask', sensor_qos)
        self._pub_debug = self.create_publisher(
            Image, '/segmentation/debug_image', sensor_qos)

        # ── Timers ────────────────────────────────────────────────────────────
        # YOLO runs at the main inference rate
        self.create_timer(1.0 / p('yolo_rate_hz'), self._run_yolo_cycle)

        # SegFormer runs at its own slower rate (surfaces are slow-changing)
        if self._segformer is not None:
            self.create_timer(1.0 / p('segformer_rate_hz'),
                              self._run_segformer_cycle)

        self.get_logger().info(
            f'SegmentationNode ready | device={device} | '
            f'YOLO @ {p("yolo_rate_hz")} Hz | '
            f'SegFormer @ {p("segformer_rate_hz")} Hz '
            f'({"enabled" if p("use_segformer") else "DISABLED"})'
        )

    # ── Parameters ────────────────────────────────────────────────────────────

    def _declare_params(self) -> None:
        d = self.declare_parameter
        # Camera
        d('image_topic',     '/camera/color/image_raw')
        # Device
        d('device',          'cuda:0')   # 'cuda:0' for RTX / Jetson, 'cpu' for test
        d('use_half',        False)      # True = FP16; recommended for Jetson Orin
        # YOLO
        d('yolo_model',      'yolov8n-seg.pt')  # or 'yolov8n-seg.engine' for TRT
        d('yolo_conf',       0.35)
        d('yolo_img_size',   640)
        d('yolo_rate_hz',    10.0)
        d('yolo_class_map', [
            0, 7,   # person      → PEDESTRIAN
            1, 9,   # bicycle     → BICYCLE
            2, 8,   # car         → VEHICLE
            3, 9,   # motorcycle  → BICYCLE
            5, 8,   # bus         → VEHICLE
            7, 8,   # truck       → VEHICLE
        ])
        # SegFormer
        d('use_segformer',      True)
        d('segformer_model',    'nvidia/segformer-b0-finetuned-cityscapes-512-1024')
        d('segformer_width',    1024)
        d('segformer_height',   512)
        d('segformer_rate_hz',  3.0)    # surfaces change slowly — 3 Hz is plenty
        # Debug
        d('publish_debug',      True)

    def _p(self, name: str):
        return self.get_parameter(name).value

    # ── Image callback ────────────────────────────────────────────────────────

    def _on_image(self, msg: Image) -> None:
        """Cache the latest camera frame (fast — no inference here)."""
        try:
            bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image decode error: {e}',
                                   throttle_duration_sec=5.0)
            return
        with self._lock:
            self._latest_bgr   = bgr
            self._latest_stamp = msg.header.stamp

    # ── Inference cycles ──────────────────────────────────────────────────────

    def _run_segformer_cycle(self) -> None:
        """
        SegFormer inference cycle.
        Runs at segformer_rate_hz (default 3 Hz).
        Updates self._surface_mask in-place.
        """
        with self._lock:
            bgr = self._latest_bgr

        if bgr is None or self._segformer is None:
            return

        t0 = time.perf_counter()
        surface_mask = self._segformer.infer(bgr)
        dt = (time.perf_counter() - t0) * 1000.0

        with self._lock:
            self._surface_mask = surface_mask

        self.get_logger().debug(f'SegFormer {dt:.1f} ms',
                                 throttle_duration_sec=2.0)

    def _run_yolo_cycle(self) -> None:
        """
        YOLO inference cycle — the main publish loop.
        Runs at yolo_rate_hz (default 10 Hz).

        1. Run YOLOv8-seg on the latest frame → object mask
        2. Merge with the cached SegFormer surface mask (objects override surfaces)
        3. Publish merged mask on /yolo/seg_mask
        """
        with self._lock:
            bgr          = self._latest_bgr
            stamp        = self._latest_stamp
            surface_mask = self._surface_mask

        if bgr is None:
            return

        t0 = time.perf_counter()

        # ── Step 1: YOLO objects ──────────────────────────────────────────────
        object_mask = self._yolo.infer(bgr)

        # ── Step 2: Merge ─────────────────────────────────────────────────────
        #   Start with surface labels (SegFormer), then paint objects on top.
        #   Objects have higher semantic priority (person > road, car > sidewalk).
        if surface_mask is not None:
            H, W = bgr.shape[:2]
            # Resize surface mask if it differs (SegFormer runs async)
            if surface_mask.shape != (H, W):
                surface_mask = cv2.resize(surface_mask, (W, H),
                                           interpolation=cv2.INTER_NEAREST)
            merged = surface_mask.copy()
            merged[object_mask > 0] = object_mask[object_mask > 0]
        else:
            # SegFormer not ready yet — publish YOLO-only mask
            merged = object_mask

        dt = (time.perf_counter() - t0) * 1000.0
        self.get_logger().debug(f'YOLO+merge {dt:.1f} ms',
                                 throttle_duration_sec=2.0)

        # ── Step 3: Publish mask ──────────────────────────────────────────────
        mask_msg = self._bridge.cv2_to_imgmsg(merged, encoding='mono8')
        mask_msg.header.stamp = stamp if stamp is not None \
                                else self.get_clock().now().to_msg()
        self._pub_mask.publish(mask_msg)

        # ── Step 4: Debug visualisation ───────────────────────────────────────
        if self._p('publish_debug'):
            debug_img = self._colorise(merged)
            debug_msg = self._bridge.cv2_to_imgmsg(debug_img, encoding='bgr8')
            debug_msg.header.stamp = mask_msg.header.stamp
            self._pub_debug.publish(debug_msg)

    # ── Helpers ───────────────────────────────────────────────────────────────

    @staticmethod
    def _colorise(mask: np.ndarray) -> np.ndarray:
        """
        Render the class-ID mask as a colour image for human inspection.
        Colours match semantic_classes.py CLASS_COLORS_BGR.
        """
        # BGR colours indexed by SemanticClass ID
        PALETTE = np.array([
            [50,  50,  50],   # 0 UNKNOWN
            [210, 210, 210],  # 1 FREE
            [30,  30,  220],  # 2 OBSTACLE
            [160, 220, 244],  # 3 SIDEWALK
            [110, 110, 110],  # 4 ROAD
            [255, 255, 255],  # 5 CROSSWALK
            [20,  160,  30],  # 6 GRASS
            [0,    0,  255],  # 7 PEDESTRIAN
            [0,  200,    0],  # 8 VEHICLE
            [0,  165,  255],  # 9 BICYCLE
            [0,  230,  230],  # 10 CURB
        ], dtype=np.uint8)

        idx = np.clip(mask, 0, len(PALETTE) - 1)
        return PALETTE[idx]

    @staticmethod
    def _parse_class_map(flat: list) -> list[tuple[int, int]]:
        """Convert flat [yolo_id, sem_id, yolo_id, sem_id, …] to list of tuples."""
        pairs = []
        for i in range(0, len(flat) - 1, 2):
            pairs.append((int(flat[i]), int(flat[i + 1])))
        return pairs


# ── Entry point ───────────────────────────────────────────────────────────────

def main(args=None) -> None:
    rclpy.init(args=args)
    node = SegmentationNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
