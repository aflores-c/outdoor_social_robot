#!/usr/bin/env python3
"""
Vehicle plate detection + registration check.

This is a separate specialized model from traffic_object_detection: COCO
YOLOv8 (used for pedestrian/vehicle detection) has no "license plate"
class, so plate localization needs its own fine-tuned detector. This node
therefore runs its own (second) YOLO pass plus OCR — unlike pedestrian and
vehicle detection, which share one COCO model, this cannot be folded into
that node.

Pipeline per frame:
  1. YOLO (plate-detector weights, NOT the COCO model) finds plate bounding
     boxes in the RGB image
  2. Each box is cropped and OCR'd (EasyOCR) to read the plate text
  3. The recognized text is normalized (uppercase, alphanumeric only) and
     checked against the `registered_plates` parameter (the school's
     allow-list)
  4. True is published as soon as any visible plate matches the allow-list;
     False otherwise (including when no plate is visible at all) — this
     fails closed, so school_traffic_control never lets a vehicle pass
     without a confirmed registered plate

Note: this node does not disambiguate between multiple vehicles/plates
simultaneously in frame — it publishes True if ANY visible plate is
registered. That matches school_traffic_control's current single-vehicle-
at-a-time crossing model.

Published topics:
  <plate_allowed_topic>                  std_msgs/Bool  (default /perception/plate_allowed
                                          — consumed by decision_making/school_traffic_control)
  /vehicle_plate_detection/last_plate    std_msgs/String  (most recent OCR'd plate text)
  /vehicle_plate_detection/debug_image   sensor_msgs/Image
"""

import re
import time

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool, String

import torch
from ultralytics import YOLO

_PLATE_CHARS_RE = re.compile(r'[^A-Z0-9]')


def normalize_plate(text: str) -> str:
    return _PLATE_CHARS_RE.sub('', text.upper())


class PlateDetectorNode(Node):

    def __init__(self):
        super().__init__('plate_detector_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('rgb_topic', '/camera/realsense2_camera/color/image_raw')
        self.declare_parameter('plate_allowed_topic', '/perception/plate_allowed')

        # Plate DETECTOR weights — a fine-tuned YOLO model, e.g. trained on a
        # license-plate dataset (Roboflow "license-plate-recognition" or
        # similar) and exported to .pt/.engine. This is NOT yolov8n.pt.
        self.declare_parameter('plate_model', 'license_plate_yolov8n.pt')
        self.declare_parameter('confidence', 0.50)

        # OCR
        self.declare_parameter('ocr_languages', ['en'])
        self.declare_parameter('ocr_min_confidence', 0.50)
        self.declare_parameter('min_plate_chars', 4)

        # The school's allow-list, e.g. ["ABC123", "XYZ789"]. Normalized
        # (uppercase, alphanumeric only) on load — see registered_plates.yaml
        self.declare_parameter('registered_plates', [''])

        # Throttle the (relatively expensive) detect+OCR pipeline
        self.declare_parameter('plate_check_rate_hz', 5.0)
        self.declare_parameter('debug_fps', 5.0)

        rgb_topic          = self.get_parameter('rgb_topic').value
        plate_allowed_topic = self.get_parameter('plate_allowed_topic').value
        model_path          = self.get_parameter('plate_model').value
        self._conf           = float(self.get_parameter('confidence').value)
        ocr_languages         = list(self.get_parameter('ocr_languages').value)
        self._ocr_min_conf    = float(self.get_parameter('ocr_min_confidence').value)
        self._min_plate_chars = int(self.get_parameter('min_plate_chars').value)

        registered = self.get_parameter('registered_plates').value
        self._registered = {normalize_plate(p) for p in registered if normalize_plate(p)}

        check_rate_hz      = float(self.get_parameter('plate_check_rate_hz').value)
        self._check_period = 1.0 / check_rate_hz if check_rate_hz > 0 else 0.0
        self._last_check_t = 0.0

        debug_fps          = float(self.get_parameter('debug_fps').value)
        self._debug_period = 1.0 / debug_fps if debug_fps > 0 else 0.0
        self._last_debug_t = 0.0

        # ── YOLO (plate detector) ────────────────────────────────────────────
        assert torch.cuda.is_available(), 'CUDA not available — check drivers'
        self._device = 'cuda'
        self._model = YOLO(model_path)
        self._model.to(self._device)

        # ── OCR ───────────────────────────────────────────────────────────────
        import easyocr
        self._reader = easyocr.Reader(ocr_languages, gpu=True)

        # ── Misc ──────────────────────────────────────────────────────────────
        self._bridge = CvBridge()

        # ── Subscriber ────────────────────────────────────────────────────────
        self.create_subscription(Image, rgb_topic, self._cb, qos_profile_sensor_data)

        # ── Publishers ────────────────────────────────────────────────────────
        self._pub_allowed = self.create_publisher(Bool, plate_allowed_topic, 10)
        self._pub_last_plate = self.create_publisher(String, '/vehicle_plate_detection/last_plate', 10)
        self._pub_debug = self.create_publisher(Image, '/vehicle_plate_detection/debug_image', 5)

        self.get_logger().info(
            f'\n{"=" * 58}\n'
            f'  Vehicle Plate Detector\n'
            f'  RGB:    {rgb_topic}\n'
            f'  Model:  {model_path}  |  conf={self._conf}\n'
            f'  GPU:    {torch.cuda.get_device_name(0)}\n'
            f'  Registered plates: {len(self._registered)}\n'
            f'  Output: {plate_allowed_topic}\n'
            f'{"=" * 58}'
        )
        if not self._registered:
            self.get_logger().warn(
                'registered_plates is empty — every detected plate will be '
                'rejected. Set it via config/registered_plates.yaml.'
            )

    # ── Main callback ──────────────────────────────────────────────────────

    def _cb(self, img_msg: Image):
        now = time.monotonic()
        if self._check_period > 0.0 and (now - self._last_check_t) < self._check_period:
            return
        self._last_check_t = now

        try:
            frame = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion: {e}', throttle_duration_sec=5.0)
            return

        h, w = frame.shape[:2]

        results = self._model(
            frame,
            conf=self._conf,
            device=self._device,
            verbose=False,
        )

        allowed = False
        last_plate = ''
        debug_img = frame.copy()

        for r in results:
            if r.boxes is None:
                continue
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()

            for (x1, y1, x2, y2), det_conf in zip(boxes, confs):
                x1 = max(int(x1), 0)
                y1 = max(int(y1), 0)
                x2 = min(int(x2), w)
                y2 = min(int(y2), h)
                if x2 <= x1 or y2 <= y1:
                    continue

                crop = frame[y1:y2, x1:x2]
                ocr_results = self._reader.readtext(crop)

                plate_text = ''
                plate_conf = 0.0
                for _, text, conf in ocr_results:
                    norm = normalize_plate(text)
                    if len(norm) >= self._min_plate_chars and conf > plate_conf:
                        plate_text = norm
                        plate_conf = conf

                is_registered = bool(plate_text) and plate_conf >= self._ocr_min_conf \
                    and plate_text in self._registered
                if plate_text:
                    last_plate = plate_text
                if is_registered:
                    allowed = True

                color = (0, 220, 0) if is_registered else (0, 0, 220)
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
                label = f'{plate_text or "?"} ({plate_conf:.2f}) det={det_conf:.2f}'
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
    node = PlateDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
