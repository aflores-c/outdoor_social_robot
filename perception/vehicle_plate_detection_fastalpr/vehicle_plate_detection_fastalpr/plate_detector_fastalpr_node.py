#!/usr/bin/env python3
"""
Vehicle plate detection + registration check, via fast-alpr.

Alternative implementation of vehicle_plate_detection/plate_detector_node.py
(YOLO + EasyOCR), kept as a fully separate package/node so both can be
built and tested side by side — this one is NOT meant to replace the
original yet. Same external interface (params, topics, registered-plate
substring matching) so it's a drop-in swap for whichever one wins.

fast-alpr (https://github.com/ankandrew/fast-alpr) replaces the custom
YOLO-detector + general-purpose-OCR pipeline with two purpose-built ONNX
models: a small end-to-end YOLOv9-t plate detector and a CCT (Compact
Convolutional Transformer) model trained specifically on plate text — not
a general scene-text reader like EasyOCR. Runs on ONNX Runtime, not
PyTorch, so this package needs its own dedicated venv with onnxruntime-gpu
instead of torch/ultralytics/easyocr — see DEPLOYMENT.md.

Pipeline per frame (all inside fast_alpr.ALPR.predict):
  1. YOLOv9-t end-to-end detector finds plate bounding boxes in the RGB image
  2. Each detected plate is OCR'd by the CCT plate-text model
  3. The recognized text is normalized (uppercase, alphanumeric only) and
     checked against the `registered_plates` parameter (the school's
     allow-list) via a bidirectional substring match, not exact equality —
     OCR often reads a few extra noise characters around the real plate
     (registered plate is a substring of the OCR text) or drops one (OCR
     text is a substring of the registered plate); see plate_matches
  4. True is published as soon as any visible plate matches the allow-list;
     False otherwise (including when no plate is visible at all) — this
     fails closed, so school_traffic_control never lets a vehicle pass
     without a confirmed registered plate. school_traffic_control also
     applies its own sliding-window vote on top of this stream, so a
     single noisy frame here doesn't flip the crossing decision either.

Note: this node does not disambiguate between multiple vehicles/plates
simultaneously in frame — it publishes True if ANY visible plate is
registered. That matches school_traffic_control's current single-vehicle-
at-a-time crossing model.

Published topics:
  <plate_allowed_topic>                  std_msgs/Bool  (default /perception/plate_allowed
                                          — consumed by decision_making/school_traffic_control)
  /vehicle_plate_detection_fastalpr/last_plate    std_msgs/String  (most recent OCR'd plate text)
  /vehicle_plate_detection_fastalpr/debug_image   sensor_msgs/Image
"""

import re
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from sensor_msgs.msg import CompressedImage, Image
from std_msgs.msg import Bool, String

from fast_alpr import ALPR

_PLATE_CHARS_RE = re.compile(r'[^A-Z0-9]')


def normalize_plate(text: str) -> str:
    return _PLATE_CHARS_RE.sub('', text.upper())


def plate_matches(plate_text: str, registered: set) -> bool:
    """Bidirectional substring match against the allow-list — see the
    module docstring for why this isn't exact equality."""
    return any(reg in plate_text or plate_text in reg for reg in registered)


def _ocr_confidence(conf) -> float:
    """OcrResult.confidence is `float | list[float]` (the list form is
    per-character confidences) — take the minimum so one weakly-read
    character can't be masked by the rest, same conservative intent as
    the original node's single scalar confidence check."""
    if isinstance(conf, (list, tuple)):
        return min(conf) if conf else 0.0
    return float(conf)


class PlateDetectorFastAlprNode(Node):

    def __init__(self):
        super().__init__('plate_detector_fastalpr_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('rgb_topic', '/head_front_camera/color/image_raw')
        self.declare_parameter('plate_allowed_topic', '/perception/plate_allowed')

        # fast-alpr's pretrained models (ONNX, downloaded/cached on first
        # use) — not a local .pt path like the original node's plate_model.
        # See https://ankandrew.github.io/fast-alpr/latest/ for alternatives.
        self.declare_parameter('detector_model', 'yolo-v9-t-384-license-plate-end2end')
        self.declare_parameter('ocr_model', 'cct-xs-v2-global-model')
        self.declare_parameter('confidence', 0.50)
        self.declare_parameter('ocr_min_confidence', 0.50)
        self.declare_parameter('min_plate_chars', 4)

        # The school's allow-list, e.g. ["ABC123", "XYZ789"]. Normalized
        # (uppercase, alphanumeric only) on load — see registered_plates.yaml
        self.declare_parameter('registered_plates', [''])

        # Throttle the detect+OCR pipeline
        self.declare_parameter('plate_check_rate_hz', 5.0)
        self.declare_parameter('debug_fps', 5.0)

        # Off by default; school_traffic_control turns this on (and
        # traffic_object_detection off) only during CHECK_PLATE.
        self.declare_parameter('enabled_topic', '/perception/plate_detection_enabled')

        rgb_topic            = self.get_parameter('rgb_topic').value
        plate_allowed_topic  = self.get_parameter('plate_allowed_topic').value
        enabled_topic         = self.get_parameter('enabled_topic').value
        detector_model        = self.get_parameter('detector_model').value
        ocr_model              = self.get_parameter('ocr_model').value
        self._conf             = float(self.get_parameter('confidence').value)
        self._ocr_min_conf     = float(self.get_parameter('ocr_min_confidence').value)
        self._min_plate_chars  = int(self.get_parameter('min_plate_chars').value)

        registered = self.get_parameter('registered_plates').value
        self._registered = {normalize_plate(p) for p in registered if normalize_plate(p)}

        check_rate_hz      = float(self.get_parameter('plate_check_rate_hz').value)
        self._check_period = 1.0 / check_rate_hz if check_rate_hz > 0 else 0.0
        self._last_check_t = 0.0

        debug_fps          = float(self.get_parameter('debug_fps').value)
        self._debug_period = 1.0 / debug_fps if debug_fps > 0 else 0.0
        self._last_debug_t = 0.0

        # ── fast-alpr (ONNX Runtime) ─────────────────────────────────────────
        # Confidence thresholds are applied by this node after predict(),
        # not passed into ALPR() itself, so behavior stays directly
        # comparable to the original node's confidence/ocr_min_confidence
        # semantics regardless of fast-alpr's own internal defaults.
        self._alpr = ALPR(detector_model=detector_model, ocr_model=ocr_model)

        # ── Misc ──────────────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── Enable/disable switch (driven by school_traffic_control) ────────
        # Defaults off: this model only runs once the state machine actually
        # needs a plate read (CHECK_PLATE), so traffic_object_detection can
        # have the GPU the rest of the time.
        self._enabled = False
        enabled_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, enabled_topic, self._on_enabled, enabled_qos)

        # ── Subscriber ────────────────────────────────────────────────────────
        # Compressed transport: on this deployment the camera stream crosses
        # wifi from the robot's onboard PC to the jetson, so subscribing to
        # <rgb_topic>/compressed (sensor_msgs/CompressedImage) instead of the
        # raw topic is what actually reaches this node. Decoded via cv_bridge
        # below (same pattern as traffic_object_detection).
        rgb_compressed_topic = rgb_topic + '/compressed'
        self.create_subscription(CompressedImage, rgb_compressed_topic, self._cb, qos_profile_sensor_data)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_allowed = self.create_publisher(Bool, plate_allowed_topic, 10)
        self._pub_last_plate = self.create_publisher(String, '/vehicle_plate_detection_fastalpr/last_plate', 10)
        self._pub_debug = self.create_publisher(Image, '/vehicle_plate_detection_fastalpr/debug_image', 5)

        self.get_logger().info(
            f'\n{"=" * 58}\n'
            f'  Vehicle Plate Detector (fast-alpr)\n'
            f'  RGB:      {rgb_compressed_topic}\n'
            f'  Detector: {detector_model}  |  conf={self._conf}\n'
            f'  OCR:      {ocr_model}  |  ocr_min_conf={self._ocr_min_conf}\n'
            f'  Registered plates: {len(self._registered)}\n'
            f'  Output:   {plate_allowed_topic}\n'
            f'{"=" * 58}'
        )
        if not self._registered:
            self.get_logger().warn(
                'registered_plates is empty — every detected plate will be '
                'rejected. Set it via config/registered_plates.yaml.'
            )

    def _on_enabled(self, msg: Bool):
        self._enabled = msg.data

    # ── Main callback ──────────────────────────────────────────────────────

    def _cb(self, img_msg: CompressedImage):
        if not self._enabled:
            return
        now = time.monotonic()
        if self._check_period > 0.0 and (now - self._last_check_t) < self._check_period:
            return
        self._last_check_t = now

        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion: {e}', throttle_duration_sec=5.0)
            return

        results = self._alpr.predict(frame)

        allowed = False
        last_plate = ''
        debug_img = frame.copy()

        for r in results:
            det = r.detection
            if det.confidence < self._conf:
                continue
            box = det.bounding_box
            x1, y1, x2, y2 = box.x1, box.y1, box.x2, box.y2

            plate_text = ''
            plate_conf = 0.0
            if r.ocr is not None:
                norm = normalize_plate(r.ocr.text)
                conf = _ocr_confidence(r.ocr.confidence)
                if len(norm) >= self._min_plate_chars:
                    plate_text = norm
                    plate_conf = conf

            is_registered = bool(plate_text) and plate_conf >= self._ocr_min_conf \
                and plate_matches(plate_text, self._registered)
            if plate_text:
                last_plate = plate_text
            if is_registered:
                allowed = True

            color = (0, 220, 0) if is_registered else (0, 0, 220)
            cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
            label = f'{plate_text or "?"} ({plate_conf:.2f}) det={det.confidence:.2f}'
            cv2.putText(debug_img, label, (x1, max(y1 - 6, 10)),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

        self._pub_allowed.publish(Bool(data=allowed))
        if last_plate:
            self._pub_last_plate.publish(String(data=last_plate))

        now_ns = self.get_clock().now().nanoseconds * 1e-9
        if self._debug_period == 0.0 or (now_ns - self._last_debug_t) >= self._debug_period:
            self._publish_debug(debug_img)
            self._last_debug_t = now_ns

    def _publish_debug(self, img):
        try:
            self._pub_debug.publish(self._bridge.cv2_to_imgmsg(img, 'bgr8'))
        except Exception:
            pass


def main(args=None):
    rclpy.init(args=args)
    node = PlateDetectorFastAlprNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
