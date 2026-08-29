#!/usr/bin/env python3
"""
Combined pedestrian + vehicle detection via a single YOLO pass + LiDAR
back-projection.

yolov8n.pt is COCO-pretrained and already recognizes person (class 0),
bicycle (class 1), and car/motorcycle/bus/truck (classes 2/3/5/7) — there
is no separate "vehicle model". Running one detector for both, instead of
one YOLO instance per object type, halves the GPU inference cost on the
Jetson Orin AGX. Bicycles are counted as vehicles for the stop/pass
decision (see BICYCLE_CLASSES) but use their own, lower LiDAR point
threshold given their much smaller physical profile.

Pipeline per frame:
  1. YOLO detects persons + vehicles in the RGB image → 2D bounding boxes
  2. LiDAR point cloud is projected into the image using the live
     velodyne → camera_frame transform (R, t looked up from TF each frame, + K)
  3. LiDAR points that fall inside each bounding box are extracted
  4. Median centroid of those points = object pose (x, y, z) in velodyne frame
  5. Detections are split by class into pedestrian / vehicle buckets
  6. The closest vehicle's distance is tracked over a rolling window
     (single-target continuity heuristic, not full tracking — see
     _update_stopped_tracking) and flagged VehicleDetection.stopped once
     it's held steady for stopped_confirm_time_s. Consumed by
     school_traffic_control to decide when to check a stopped vehicle's
     plate instead of just holding position indefinitely.

The camera → LiDAR transform is read from the TF tree on every frame rather than
loaded once from a calibration YAML, so this stays correct if the camera is on a
moving joint (e.g. a pan/tilt head) — the transform updates automatically as the
joint moves, as long as both frames are connected in the URDF/TF tree.

Published topics:
  <pedestrians_topic>                    traffic_perception_msgs/PedestrianDetectionArray
                                          (default /perception/pedestrians)
  <vehicles_topic>                       traffic_perception_msgs/VehicleDetectionArray
                                          (default /perception/vehicles)
  /traffic_object_detection/pedestrians/poses    geometry_msgs/PoseArray   (velodyne frame)
  /traffic_object_detection/pedestrians/markers  visualization_msgs/MarkerArray
  /traffic_object_detection/vehicles/poses       geometry_msgs/PoseArray   (velodyne frame)
  /traffic_object_detection/vehicles/markers     visualization_msgs/MarkerArray
  /traffic_object_detection/debug_image          sensor_msgs/Image  (LiDAR-confirmed boxes; dropped
                                          ones shown gray/"no LiDAR")
  /traffic_object_detection/debug_image_yolo_only  sensor_msgs/Image  (raw YOLO boxes, no LiDAR
                                          fusion involved — debug only)
"""

import threading
import time
from collections import deque

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy, qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, CompressedImage, Image, PointCloud2
from std_msgs.msg import Bool
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener
from traffic_perception_msgs.msg import (
    PedestrianDetection, PedestrianDetectionArray,
    VehicleDetection, VehicleDetectionArray,
)
from visualization_msgs.msg import Marker, MarkerArray

import torch
from ultralytics import YOLO

# COCO class ids
PEDESTRIAN_CLASSES = [0]
VEHICLE_CLASSES = [2, 3, 5, 7]   # car, motorcycle, bus, truck
BICYCLE_CLASSES = [1]           # bicycle — treated as a vehicle for stop/pass
                                 # decisions, but with its own (lower) LiDAR
                                 # point threshold: much smaller profile than
                                 # a car, so vehicle_min_lidar_points would
                                 # drop real detections.
ALL_CLASSES = PEDESTRIAN_CLASSES + VEHICLE_CLASSES + BICYCLE_CLASSES

COCO_CLASS_NAMES = {0: 'person', 1: 'bicycle', 2: 'car', 3: 'motorcycle', 5: 'bus', 7: 'truck'}


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


class TrafficObjectDetectorNode(Node):

    def __init__(self):
        super().__init__('traffic_object_detector_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('rgb_topic',         '/camera/realsense2_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/realsense2_camera/color/camera_info')
        self.declare_parameter('lidar_topic',       '/velodyne_points')
        self.declare_parameter('camera_frame',      'camera_color_optical_frame')
        self.declare_parameter('yolo_model',        'yolov8n.pt')
        self.declare_parameter('confidence',        0.40)
        self.declare_parameter('pedestrian_min_lidar_points', 5)
        self.declare_parameter('vehicle_min_lidar_points',    10)
        self.declare_parameter('bicycle_min_lidar_points',    7)
        # "Vehicle stopped" signal: single-target continuity heuristic (the
        # closest vehicle each frame, no full multi-object tracking) — the
        # closest vehicle's distance is tracked over a rolling window and
        # flagged stopped once it's stayed within stopped_distance_epsilon_m
        # for stopped_confirm_time_s. stopped_association_distance_m bounds
        # how far the closest distance can jump frame-to-frame before it's
        # treated as a different vehicle (history resets).
        self.declare_parameter('stopped_confirm_time_s',        1.5)
        self.declare_parameter('stopped_distance_epsilon_m',    0.3)
        self.declare_parameter('stopped_association_distance_m', 2.0)
        self.declare_parameter('sync_slop_s',       0.10)
        self.declare_parameter('lidar_frame',       'velodyne')
        self.declare_parameter('max_range_m',       20.0)
        self.declare_parameter('debug_fps',         5.0)
        self.declare_parameter('pedestrians_topic', '/perception/pedestrians')
        self.declare_parameter('vehicles_topic',    '/perception/vehicles')
        # velodyne mounted ~1.74m above ground (frame is Z-up); points within
        # ground_margin_m of that plane are treated as ground clutter, not object.
        self.declare_parameter('ground_height_m',   1.74)
        self.declare_parameter('ground_margin_m',   0.15)
        self.declare_parameter('outlier_mad_k',     3.0)
        # Runs by default; school_traffic_control turns this off (and
        # vehicle_plate_detection on) while it's in VEHICLE_STOP, since
        # running both heavy models at once is too much for one Jetson.
        self.declare_parameter('enabled_topic', '/perception/traffic_object_detection_enabled')

        rgb_topic   = self.get_parameter('rgb_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        pedestrians_topic = self.get_parameter('pedestrians_topic').value
        vehicles_topic    = self.get_parameter('vehicles_topic').value
        enabled_topic     = self.get_parameter('enabled_topic').value
        self._camera_frame = self.get_parameter('camera_frame').value
        model_path  = self.get_parameter('yolo_model').value
        self._conf              = float(self.get_parameter('confidence').value)
        self._min_pts_ped       = int(self.get_parameter('pedestrian_min_lidar_points').value)
        self._min_pts_veh       = int(self.get_parameter('vehicle_min_lidar_points').value)
        self._min_pts_bike      = int(self.get_parameter('bicycle_min_lidar_points').value)
        self._stopped_confirm_time        = float(self.get_parameter('stopped_confirm_time_s').value)
        self._stopped_distance_epsilon    = float(self.get_parameter('stopped_distance_epsilon_m').value)
        self._stopped_association_distance = float(self.get_parameter('stopped_association_distance_m').value)
        self._lidar_frame       = self.get_parameter('lidar_frame').value
        self._max_range         = float(self.get_parameter('max_range_m').value)
        self._ground_z          = -(float(self.get_parameter('ground_height_m').value)
                                     - float(self.get_parameter('ground_margin_m').value))
        self._outlier_mad_k     = float(self.get_parameter('outlier_mad_k').value)
        debug_fps                = float(self.get_parameter('debug_fps').value)
        self._debug_period       = 1.0 / debug_fps if debug_fps > 0 else 0.0
        self._last_debug_t       = 0.0

        # ── Camera <-> LiDAR transform (read live from TF every frame) ──────────
        # Not loaded once from a calibration YAML: if camera_frame is on a moving
        # joint (e.g. a pan/tilt head), a fixed transform goes stale the moment it
        # moves. TF already tracks that live via /joint_states, as long as
        # camera_frame and lidar_frame are connected in the URDF.
        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        # ── YOLO ──────────────────────────────────────────────────────────────
        assert torch.cuda.is_available(), 'CUDA not available — check drivers'
        self._device = 'cuda'
        self._model = YOLO(model_path)
        self._model.to(self._device)

        # ── Misc ──────────────────────────────────────────────────────────────
        self._bridge = CvBridge()
        self._lock   = threading.Lock()

        # ── "Vehicle stopped" tracking state ─────────────────────────────────
        self._stopped_history = deque()   # (t_monotonic, distance) — closest vehicle only
        self._stopped_last_distance = None
        self._closest_vehicle_stopped = False

        # ── Enable/disable switch (driven by school_traffic_control) ────────
        # Defaults on: this model runs unless explicitly told to stand down
        # (while vehicle_plate_detection needs the GPU instead).
        self._enabled = True
        enabled_qos = QoSProfile(
            depth=1,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Bool, enabled_topic, self._on_enabled, enabled_qos)

        # ── Subscribers (synchronized) ─────────────────────────────────────
        # Compressed transport for RGB: on this deployment the camera stream
        # crosses wifi from the robot's onboard PC to the jetson, so subscribing
        # to <rgb_topic>/compressed (sensor_msgs/CompressedImage) instead of the
        # raw topic cuts that leg's bandwidth. Decoded via cv_bridge below.
        rgb_compressed_topic = rgb_topic + '/compressed'
        slop = float(self.get_parameter('sync_slop_s').value)
        self._sub_img  = Subscriber(self, CompressedImage, rgb_compressed_topic, qos_profile=qos_profile_sensor_data)
        self._sub_info = Subscriber(self, CameraInfo,  info_topic,  qos_profile=qos_profile_sensor_data)
        self._sub_pc   = Subscriber(self, PointCloud2, lidar_topic, qos_profile=qos_profile_sensor_data)
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_img, self._sub_info, self._sub_pc],
            queue_size=5, slop=slop,
        )
        self._sync.registerCallback(self._cb)

        # ── DEBUG: raw per-topic arrival counters, independent of the
        # synchronizer, to tell apart "topic not reaching this node" from
        # "reaching it but never matched into a triple". Remove once resolved.
        self._dbg_counts = {'img': 0, 'info': 0, 'pc': 0, 'cb': 0}
        self.create_subscription(CompressedImage, rgb_compressed_topic, self._dbg_image_only, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, lambda m: self._dbg_bump('info'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, lidar_topic, lambda m: self._dbg_bump('pc'), qos_profile_sensor_data)
        self.create_timer(2.0, self._dbg_report)

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_ped_poses   = self.create_publisher(PoseArray, '/traffic_object_detection/pedestrians/poses', 10)
        self._pub_ped_markers = self.create_publisher(MarkerArray, '/traffic_object_detection/pedestrians/markers', 10)
        self._pub_veh_poses   = self.create_publisher(PoseArray, '/traffic_object_detection/vehicles/poses', 10)
        self._pub_veh_markers = self.create_publisher(MarkerArray, '/traffic_object_detection/vehicles/markers', 10)
        self._pub_debug       = self.create_publisher(Image, '/traffic_object_detection/debug_image', 5)
        # Raw YOLO boxes only, no LiDAR fusion/sync/TF involved — lets you
        # confirm what the detector alone sees, decoupled from whether
        # anything gets confirmed/dropped by the LiDAR point-count filter.
        self._pub_debug_yolo_only = self.create_publisher(
            Image, '/traffic_object_detection/debug_image_yolo_only', 5)

        self._pub_pedestrians = self.create_publisher(PedestrianDetectionArray, pedestrians_topic, 10)
        self._pub_vehicles    = self.create_publisher(VehicleDetectionArray, vehicles_topic, 10)

        self.get_logger().info(
            f'\n{"=" * 58}\n'
            f'  Traffic Object Detector (pedestrians + vehicles)\n'
            f'  RGB:   {rgb_compressed_topic}\n'
            f'  LiDAR: {lidar_topic}\n'
            f'  Model: {model_path}  |  conf={self._conf}  |  classes={ALL_CLASSES}\n'
            f'  GPU:   {torch.cuda.get_device_name(0)}\n'
            f'  TF:    {self._camera_frame} -> {self._lidar_frame} (looked up live)\n'
            f'{"=" * 58}'
        )

    def _on_enabled(self, msg: Bool):
        self._enabled = msg.data

    # ── DEBUG helpers ────────────────────────────────────────────────────
    def _dbg_bump(self, key):
        self._dbg_counts[key] += 1

    def _dbg_image_only(self, img_msg: CompressedImage):
        """YOLO on every image frame, no LiDAR/sync/TF involved — isolates
        whether the detector alone finds something, decoupled from whether
        the LiDAR point-count filter later confirms/drops it. Publishes its
        own debug image (boxes + class + confidence, no LiDAR overlay) to
        /traffic_object_detection/debug_image_yolo_only."""
        if not self._enabled:
            return
        self._dbg_bump('img')
        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'[DEBUG image-only] conversion failed: {e}', throttle_duration_sec=5.0)
            return
        results = self._model(frame, conf=self._conf, classes=ALL_CLASSES, device=self._device, verbose=False)

        by_class = {}
        debug_img = frame
        for r in results:
            if r.boxes is None:
                continue
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            clses = r.boxes.cls.cpu().numpy().astype(int)
            for (x1, y1, x2, y2), conf, cls_id in zip(boxes, confs, clses):
                name = COCO_CLASS_NAMES.get(int(cls_id), str(int(cls_id)))
                by_class[name] = by_class.get(name, 0) + 1
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 200, 255), 2)
                cv2.putText(debug_img, f'{name} {conf:.2f}', (x1, max(y1 - 6, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 200, 255), 2)

        n = sum(by_class.values())
        breakdown = ' '.join(f'{k}={v}' for k, v in sorted(by_class.items())) or 'none'
        self.get_logger().info(f'[DEBUG image-only] yolo_boxes={n} ({breakdown})', throttle_duration_sec=1.0)

        try:
            self._pub_debug_yolo_only.publish(self._bridge.cv2_to_imgmsg(debug_img, 'bgr8'))
        except Exception:
            pass

    def _dbg_report(self):
        c = self._dbg_counts
        self.get_logger().info(
            f'[DEBUG] last 2s — img:{c["img"]} info:{c["info"]} pc:{c["pc"]} synced_cb:{c["cb"]}'
        )
        for k in c:
            c[k] = 0

    # ── Main callback ──────────────────────────────────────────────────────

    def _cb(self, img_msg: CompressedImage, info_msg: CameraInfo, pc_msg: PointCloud2):
        if not self._enabled:
            return
        self._dbg_counts['cb'] += 1
        # Camera intrinsics
        K = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(info_msg.d, dtype=np.float64)

        try:
            frame = self._bridge.compressed_imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion: {e}', throttle_duration_sec=5.0)
            return

        h, w = frame.shape[:2]

        # ── Live camera <-> LiDAR transform ──────────────────────────────────
        try:
            tf = self._tf_buffer.lookup_transform(self._camera_frame, self._lidar_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(f'TF lookup {self._camera_frame} <- {self._lidar_frame} failed: {e}',
                                    throttle_duration_sec=5.0)
            return
        tr = tf.transform.translation
        q  = tf.transform.rotation
        R = _quat_to_matrix(q.x, q.y, q.z, q.w)
        t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)

        # ── LiDAR → camera projection ──────────────────────────────────────
        pts_lidar = self._pc2_to_xyz(pc_msg)

        # Range filter
        ranges = np.linalg.norm(pts_lidar, axis=1)
        pts_lidar = pts_lidar[ranges < self._max_range]

        # Transform to camera optical frame
        pts_cam = (R @ pts_lidar.T).T + t

        # Keep only points in front of camera
        front = pts_cam[:, 2] > 0.1
        pts_cam   = pts_cam[front]
        pts_lidar = pts_lidar[front]

        if len(pts_cam) == 0:
            return

        # Project to pixel coords (with lens distortion)
        if D is not None and len(D) > 0 and np.any(D != 0):
            img_pts, _ = cv2.projectPoints(
                pts_cam.reshape(-1, 1, 3).astype(np.float32),
                np.zeros(3), np.zeros(3),
                K.astype(np.float32), D.astype(np.float32),
            )
            px = img_pts[:, 0, 0]
            py = img_pts[:, 0, 1]
        else:
            fx, fy = K[0, 0], K[1, 1]
            cx, cy = K[0, 2], K[1, 2]
            px = (fx * pts_cam[:, 0] / pts_cam[:, 2] + cx).astype(np.float32)
            py = (fy * pts_cam[:, 1] / pts_cam[:, 2] + cy).astype(np.float32)

        # Keep in-image points
        in_img = (px >= 0) & (px < w) & (py >= 0) & (py < h)
        px        = px[in_img]
        py        = py[in_img]
        pts_lidar = pts_lidar[in_img]
        depths    = pts_cam[in_img, 2]

        # ── YOLO detection — single pass for both object types ─────────────
        results = self._model(
            frame,
            conf=self._conf,
            classes=ALL_CLASSES,
            device=self._device,
            verbose=False,
        )

        total_boxes = sum(len(r.boxes) if r.boxes is not None else 0 for r in results)
        self.get_logger().info(
            f'[DEBUG] cb fired: lidar_pts_in_img={len(pts_lidar)} yolo_boxes={total_boxes}',
            throttle_duration_sec=2.0,
        )

        # ── Per-detection pose extraction ──────────────────────────────────
        poses_ped = []   # (x, y, z, conf) in velodyne frame
        poses_veh = []
        debug_img = frame.copy()
        stamp     = img_msg.header.stamp

        for r in results:
            if r.boxes is None:
                continue
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()
            clses = r.boxes.cls.cpu().numpy().astype(int)

            for (x1, y1, x2, y2), conf, cls_id in zip(boxes, confs, clses):
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)
                is_bicycle = cls_id in BICYCLE_CLASSES
                # Bicycles count as vehicles for the stop/pass decision
                # (published into the same vehicles_topic/poses_veh), but
                # get their own (lower) LiDAR point threshold below — much
                # smaller profile than a car.
                is_vehicle = is_bicycle or cls_id in VEHICLE_CLASSES
                if is_bicycle:
                    min_pts = self._min_pts_bike
                elif is_vehicle:
                    min_pts = self._min_pts_veh
                else:
                    min_pts = self._min_pts_ped
                color   = (0, 140, 255) if is_vehicle else (0, 220, 0)   # orange / green
                kind    = 'bicycle' if is_bicycle else ('vehicle' if is_vehicle else 'pedestrian')

                # LiDAR points inside this bounding box
                inside = (px >= x1) & (px <= x2) & (py >= y1) & (py <= y2)
                pts_in = pts_lidar[inside]
                n_raw = len(pts_in)

                # Ground removal: drop points at/near the flat ground plane
                # (velodyne frame is Z-up) before they can pull the centroid down.
                pts_in = pts_in[pts_in[:, 2] > self._ground_z]
                n_after_ground = len(pts_in)

                # Outlier rejection: reject points whose range is far from the
                # box's robust median range (median absolute deviation, scaled to
                # approximate std dev) — catches stray background/reflection
                # points inside an otherwise loose bounding box.
                if len(pts_in) > 0:
                    ranges_in = np.linalg.norm(pts_in, axis=1)
                    med_range = np.median(ranges_in)
                    scaled_mad = max(np.median(np.abs(ranges_in - med_range)) * 1.4826, 0.05)
                    pts_in = pts_in[np.abs(ranges_in - med_range) <= self._outlier_mad_k * scaled_mad]

                if len(pts_in) < min_pts:
                    self.get_logger().info(
                        f'[DEBUG] {kind} box conf={conf:.2f} bbox=({x1},{y1},{x2},{y2}) '
                        f'pts_in={n_raw} after_ground={n_after_ground} after_outliers={len(pts_in)} '
                        f'< min_pts={min_pts} -> dropped',
                        throttle_duration_sec=2.0,
                    )
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (100, 100, 100), 2)
                    cv2.putText(debug_img, 'no LiDAR', (x1, y1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                    continue

                # Median centroid in velodyne frame (robust to remaining outliers)
                cx3d, cy3d, cz3d = np.median(pts_in, axis=0)
                dist = float(np.sqrt(cx3d**2 + cy3d**2))
                (poses_veh if is_vehicle else poses_ped).append((cx3d, cy3d, cz3d, conf))

                # ── Debug overlay ──────────────────────────────────────────
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), color, 2)
                label = f'{kind} {conf:.2f} | ({cx3d:.1f},{cy3d:.1f},{cz3d:.1f})m  d={dist:.1f}m'
                cv2.putText(debug_img, label, (x1, max(y1 - 6, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

                # Draw LiDAR points in box, coloured by depth
                d_in  = depths[inside]
                d_min, d_max = float(d_in.min()), float(d_in.max())
                px_in = px[inside].astype(np.int32)
                py_in = py[inside].astype(np.int32)
                for i in range(len(px_in)):
                    t_norm = float(np.clip((d_in[i] - d_min) / (d_max - d_min + 1e-6), 0, 1))
                    b = int(255 * (1.0 - t_norm))
                    rv = int(255 * t_norm)
                    cv2.circle(debug_img, (px_in[i], py_in[i]), 2, (b, 0, rv), -1)

                # Centroid pixel (project back)
                cp_cam = R @ np.array([cx3d, cy3d, cz3d]) + t
                if cp_cam[2] > 0:
                    u = int(K[0, 0] * cp_cam[0] / cp_cam[2] + K[0, 2])
                    v = int(K[1, 1] * cp_cam[1] / cp_cam[2] + K[1, 2])
                    if 0 <= u < w and 0 <= v < h:
                        cv2.drawMarker(debug_img, (u, v), (0, 255, 255),
                                       cv2.MARKER_CROSS, 16, 2)

        # ── Publish ────────────────────────────────────────────────────────
        self._publish_poses(self._pub_ped_poses, poses_ped, stamp)
        self._publish_markers(self._pub_ped_markers, poses_ped, stamp, 'pedestrian', (1.0, 0.3, 0.0), Marker.SPHERE, (0.5, 0.5, 0.5))
        self._publish_pedestrians(poses_ped, stamp)

        self._publish_poses(self._pub_veh_poses, poses_veh, stamp)
        self._publish_markers(self._pub_veh_markers, poses_veh, stamp, 'vehicle', (1.0, 0.55, 0.0), Marker.CUBE, (2.0, 1.0, 0.8))
        self._update_stopped_tracking(poses_veh)
        self._publish_vehicles(poses_veh, stamp)

        # Throttle debug image to save network bandwidth (Jetson → PC)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._debug_period == 0.0 or (now - self._last_debug_t) >= self._debug_period:
            self._publish_debug(debug_img, stamp)
            self._last_debug_t = now

    # ── Publishers ─────────────────────────────────────────────────────────

    def _publish_poses(self, pub, poses_3d, stamp):
        msg = PoseArray()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._lidar_frame
        for x, y, z, _ in poses_3d:
            p = Pose()
            p.position.x = float(x)
            p.position.y = float(y)
            p.position.z = float(z)
            p.orientation.w = 1.0
            msg.poses.append(p)
        pub.publish(msg)

    def _publish_markers(self, pub, poses_3d, stamp, ns, rgb, shape, scale):
        msg = MarkerArray()

        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        msg.markers.append(del_marker)

        for i, (x, y, z, conf) in enumerate(poses_3d):
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = self._lidar_frame
            m.ns     = ns
            m.id     = i
            m.type   = shape
            m.action = Marker.ADD
            m.pose.position.x  = float(x)
            m.pose.position.y  = float(y)
            m.pose.position.z  = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x, m.scale.y, m.scale.z = scale
            m.color.r, m.color.g, m.color.b = rgb
            m.color.a = 0.8
            m.lifetime.sec = 1
            msg.markers.append(m)

            txt = Marker()
            txt.header.stamp    = stamp
            txt.header.frame_id = self._lidar_frame
            txt.ns     = f'{ns}_label'
            txt.id     = i
            txt.type   = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x  = float(x)
            txt.pose.position.y  = float(y)
            txt.pose.position.z  = float(z) + scale[2] + 0.4
            txt.pose.orientation.w = 1.0
            txt.scale.z  = 0.3
            txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
            dist = float(np.sqrt(x**2 + y**2))
            txt.text     = f'{dist:.1f}m  ({x:.1f},{y:.1f},{z:.1f})'
            txt.lifetime.sec = 1
            msg.markers.append(txt)

        pub.publish(msg)

    def _publish_pedestrians(self, poses_3d, stamp):
        msg = PedestrianDetectionArray()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._lidar_frame
        for x, y, z, conf in poses_3d:
            d = PedestrianDetection()
            d.id = -1  # no persistent tracking
            d.distance = float(np.sqrt(x**2 + y**2))
            d.position = Point(x=float(x), y=float(y), z=float(z))
            d.confidence = float(conf)
            msg.pedestrians.append(d)
        self._pub_pedestrians.publish(msg)

    def _update_stopped_tracking(self, poses_veh):
        """Single-target continuity heuristic: track the closest vehicle's
        distance over a rolling window and flag it stopped once that
        distance has stayed within stopped_distance_epsilon_m for
        stopped_confirm_time_s. Only the closest vehicle each frame is
        tracked (no full multi-object tracking), matching
        school_traffic_control's single-vehicle-at-a-time crossing model."""
        now = time.monotonic()

        if not poses_veh:
            self._stopped_history.clear()
            self._stopped_last_distance = None
            self._closest_vehicle_stopped = False
            return

        closest = min(float(np.sqrt(x**2 + y**2)) for x, y, z, _ in poses_veh)

        if (self._stopped_last_distance is None
                or abs(closest - self._stopped_last_distance) > self._stopped_association_distance):
            # First sighting, or the closest distance jumped too far to
            # still be the same vehicle — restart history.
            self._stopped_history.clear()
        self._stopped_last_distance = closest

        self._stopped_history.append((now, closest))
        # Bound memory generously beyond the confirm window; the "stopped"
        # decision below only looks at entries within stopped_confirm_time_s,
        # not this whole buffer.
        max_age = self._stopped_confirm_time * 3.0
        while self._stopped_history and (now - self._stopped_history[0][0]) > max_age:
            self._stopped_history.popleft()

        window = [(t, d) for t, d in self._stopped_history if (now - t) <= self._stopped_confirm_time]
        if not window:
            self._closest_vehicle_stopped = False
            return
        covers_window = (now - window[0][0]) >= self._stopped_confirm_time * 0.9
        window_dists = [d for _, d in window]
        stable = (max(window_dists) - min(window_dists)) <= self._stopped_distance_epsilon
        self._closest_vehicle_stopped = bool(covers_window and stable)

    def _publish_vehicles(self, poses_3d, stamp):
        msg = VehicleDetectionArray()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._lidar_frame

        closest_idx = None
        if poses_3d:
            dists = [float(np.sqrt(x**2 + y**2)) for x, y, z, _ in poses_3d]
            closest_idx = int(np.argmin(dists))

        for i, (x, y, z, conf) in enumerate(poses_3d):
            d = VehicleDetection()
            d.id = -1  # no persistent tracking
            d.distance = float(np.sqrt(x**2 + y**2))
            d.position = Point(x=float(x), y=float(y), z=float(z))
            d.confidence = float(conf)
            # Only the closest vehicle is tracked for "stopped" (see
            # _update_stopped_tracking) — others in frame simultaneously
            # always read False.
            d.stopped = bool(i == closest_idx and self._closest_vehicle_stopped)
            msg.vehicles.append(d)
        self._pub_vehicles.publish(msg)

    def _publish_debug(self, img, stamp):
        try:
            self._pub_debug.publish(self._bridge.cv2_to_imgmsg(img, 'bgr8'))
        except Exception:
            pass

    # ── PointCloud2 → XYZ ─────────────────────────────────────────────────

    def _pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        off  = {f.name: f.offset for f in msg.fields}
        n    = msg.width * msg.height
        step = msg.point_step
        buf  = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
        ox, oy, oz = off['x'], off['y'], off['z']
        xs = buf[:, ox:ox + 4].copy().view(np.float32).reshape(-1)
        ys = buf[:, oy:oy + 4].copy().view(np.float32).reshape(-1)
        zs = buf[:, oz:oz + 4].copy().view(np.float32).reshape(-1)
        pts = np.column_stack([xs, ys, zs]).astype(np.float64)
        return pts[np.isfinite(pts).all(axis=1)]


def main(args=None):
    rclpy.init(args=args)
    node = TrafficObjectDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
