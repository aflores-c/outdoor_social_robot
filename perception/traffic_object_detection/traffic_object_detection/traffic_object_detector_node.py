#!/usr/bin/env python3
"""
Combined pedestrian + vehicle detection via a single YOLO pass + LiDAR
back-projection.

yolov8n.pt is COCO-pretrained and already recognizes person (class 0) and
car/motorcycle/bus/truck (classes 2/3/5/7) — there is no separate "vehicle
model". Running one detector for both, instead of one YOLO instance per
object type, halves the GPU inference cost on the Jetson Orin AGX.

Pipeline per frame:
  1. YOLO detects persons + vehicles in the RGB image → 2D bounding boxes
  2. LiDAR point cloud is projected into the image using the live
     velodyne → camera_frame transform (R, t looked up from TF each frame, + K)
  3. LiDAR points that fall inside each bounding box are extracted
  4. Median centroid of those points = object pose (x, y, z) in velodyne frame
  5. Detections are split by class into pedestrian / vehicle buckets

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
  /traffic_object_detection/debug_image          sensor_msgs/Image
"""

import threading

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import Point, Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
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
ALL_CLASSES = PEDESTRIAN_CLASSES + VEHICLE_CLASSES


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
        self.declare_parameter('sync_slop_s',       0.10)
        self.declare_parameter('lidar_frame',       'velodyne')
        self.declare_parameter('max_range_m',       20.0)
        self.declare_parameter('debug_fps',         5.0)
        self.declare_parameter('pedestrians_topic', '/perception/pedestrians')
        self.declare_parameter('vehicles_topic',    '/perception/vehicles')

        rgb_topic   = self.get_parameter('rgb_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        pedestrians_topic = self.get_parameter('pedestrians_topic').value
        vehicles_topic    = self.get_parameter('vehicles_topic').value
        self._camera_frame = self.get_parameter('camera_frame').value
        model_path  = self.get_parameter('yolo_model').value
        self._conf              = float(self.get_parameter('confidence').value)
        self._min_pts_ped       = int(self.get_parameter('pedestrian_min_lidar_points').value)
        self._min_pts_veh       = int(self.get_parameter('vehicle_min_lidar_points').value)
        self._lidar_frame       = self.get_parameter('lidar_frame').value
        self._max_range         = float(self.get_parameter('max_range_m').value)
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

        # ── Subscribers (synchronized) ─────────────────────────────────────
        slop = float(self.get_parameter('sync_slop_s').value)
        self._sub_img  = Subscriber(self, Image,       rgb_topic,   qos_profile=qos_profile_sensor_data)
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
        self.create_subscription(Image, rgb_topic, lambda m: self._dbg_bump('img'), qos_profile_sensor_data)
        self.create_subscription(CameraInfo, info_topic, lambda m: self._dbg_bump('info'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, lidar_topic, lambda m: self._dbg_bump('pc'), qos_profile_sensor_data)
        self.create_timer(2.0, self._dbg_report)

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_ped_poses   = self.create_publisher(PoseArray, '/traffic_object_detection/pedestrians/poses', 10)
        self._pub_ped_markers = self.create_publisher(MarkerArray, '/traffic_object_detection/pedestrians/markers', 10)
        self._pub_veh_poses   = self.create_publisher(PoseArray, '/traffic_object_detection/vehicles/poses', 10)
        self._pub_veh_markers = self.create_publisher(MarkerArray, '/traffic_object_detection/vehicles/markers', 10)
        self._pub_debug       = self.create_publisher(Image, '/traffic_object_detection/debug_image', 5)

        self._pub_pedestrians = self.create_publisher(PedestrianDetectionArray, pedestrians_topic, 10)
        self._pub_vehicles    = self.create_publisher(VehicleDetectionArray, vehicles_topic, 10)

        self.get_logger().info(
            f'\n{"=" * 58}\n'
            f'  Traffic Object Detector (pedestrians + vehicles)\n'
            f'  RGB:   {rgb_topic}\n'
            f'  LiDAR: {lidar_topic}\n'
            f'  Model: {model_path}  |  conf={self._conf}  |  classes={ALL_CLASSES}\n'
            f'  GPU:   {torch.cuda.get_device_name(0)}\n'
            f'  TF:    {self._camera_frame} -> {self._lidar_frame} (looked up live)\n'
            f'{"=" * 58}'
        )

    # ── DEBUG helpers ────────────────────────────────────────────────────
    def _dbg_bump(self, key):
        self._dbg_counts[key] += 1

    def _dbg_report(self):
        c = self._dbg_counts
        self.get_logger().info(
            f'[DEBUG] last 2s — img:{c["img"]} info:{c["info"]} pc:{c["pc"]} synced_cb:{c["cb"]}'
        )
        for k in c:
            c[k] = 0

    # ── Main callback ──────────────────────────────────────────────────────

    def _cb(self, img_msg: Image, info_msg: CameraInfo, pc_msg: PointCloud2):
        self._dbg_counts['cb'] += 1
        # Camera intrinsics
        K = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(info_msg.d, dtype=np.float64)

        try:
            frame = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
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
                is_vehicle = cls_id in VEHICLE_CLASSES
                min_pts = self._min_pts_veh if is_vehicle else self._min_pts_ped
                color   = (0, 140, 255) if is_vehicle else (0, 220, 0)   # orange / green
                kind    = 'vehicle' if is_vehicle else 'pedestrian'

                # LiDAR points inside this bounding box
                inside = (px >= x1) & (px <= x2) & (py >= y1) & (py <= y2)
                pts_in = pts_lidar[inside]

                if len(pts_in) < min_pts:
                    self.get_logger().info(
                        f'[DEBUG] {kind} box conf={conf:.2f} bbox=({x1},{y1},{x2},{y2}) '
                        f'pts_in={len(pts_in)} < min_pts={min_pts} -> dropped',
                        throttle_duration_sec=2.0,
                    )
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (100, 100, 100), 2)
                    cv2.putText(debug_img, 'no LiDAR', (x1, y1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                    continue

                # Median centroid in velodyne frame (robust to outliers)
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
        for x, y, z, _ in poses_3d:
            d = PedestrianDetection()
            d.id = -1  # no persistent tracking
            d.distance = float(np.sqrt(x**2 + y**2))
            d.position = Point(x=float(x), y=float(y), z=float(z))
            msg.pedestrians.append(d)
        self._pub_pedestrians.publish(msg)

    def _publish_vehicles(self, poses_3d, stamp):
        msg = VehicleDetectionArray()
        msg.header.stamp    = stamp
        msg.header.frame_id = self._lidar_frame
        for x, y, z, _ in poses_3d:
            d = VehicleDetection()
            d.id = -1  # no persistent tracking
            d.distance = float(np.sqrt(x**2 + y**2))
            d.position = Point(x=float(x), y=float(y), z=float(z))
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
