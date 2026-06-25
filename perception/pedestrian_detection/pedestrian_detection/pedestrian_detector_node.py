#!/usr/bin/env python3
"""
Pedestrian detection + 3D pose estimation via YOLO + LiDAR back-projection.

Pipeline per frame:
  1. YOLO detects persons in RGB image → 2D bounding boxes
  2. LiDAR point cloud is projected into the image using the calibrated
     velodyne → camera_color_optical_frame transform (R, t, K)
  3. LiDAR points that fall inside each bounding box are extracted
  4. Median centroid of those points = pedestrian pose (x, y, z) in velodyne frame

Published topics:
  /pedestrian_detection/poses        geometry_msgs/PoseArray   (velodyne frame)
  /pedestrian_detection/markers      visualization_msgs/MarkerArray
  /pedestrian_detection/debug_image  sensor_msgs/Image
"""

import threading
from pathlib import Path

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose, PoseArray
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from visualization_msgs.msg import Marker, MarkerArray

import torch
from ultralytics import YOLO


class PedestrianDetectorNode(Node):

    def __init__(self):
        super().__init__('pedestrian_detector_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('rgb_topic',         '/camera/realsense2_camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/realsense2_camera/color/camera_info')
        self.declare_parameter('lidar_topic',       '/velodyne_points')
        self.declare_parameter('calibration_file',  '')
        self.declare_parameter('yolo_model',        'yolov8n.pt')
        self.declare_parameter('confidence',        0.40)
        self.declare_parameter('min_lidar_points',  5)
        self.declare_parameter('sync_slop_s',       0.10)
        self.declare_parameter('lidar_frame',       'velodyne')
        self.declare_parameter('max_range_m',       20.0)
        self.declare_parameter('debug_fps',         5.0)

        rgb_topic   = self.get_parameter('rgb_topic').value
        info_topic  = self.get_parameter('camera_info_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        cal_file    = self.get_parameter('calibration_file').value
        model_path  = self.get_parameter('yolo_model').value
        self._conf          = float(self.get_parameter('confidence').value)
        self._min_pts       = int(self.get_parameter('min_lidar_points').value)
        self._lidar_frame   = self.get_parameter('lidar_frame').value
        self._max_range     = float(self.get_parameter('max_range_m').value)
        debug_fps           = float(self.get_parameter('debug_fps').value)
        self._debug_period  = 1.0 / debug_fps if debug_fps > 0 else 0.0
        self._last_debug_t  = 0.0

        # ── Calibration ───────────────────────────────────────────────────────
        if not cal_file:
            cal_file = str(
                Path.home() / '.ros' / 'lidar_camera_calibration' / 'lidar_to_camera.yaml'
            )
        cal_path = Path(cal_file)
        if not cal_path.exists():
            self.get_logger().error(
                f'Calibration file not found: {cal_path}\n'
                'Run: ros2 launch lidar_camera_calibration collect.launch.py'
            )
            raise FileNotFoundError(str(cal_path))

        with open(cal_path) as f:
            cal = yaml.safe_load(f)['lidar_to_camera']
        self._R = np.array(cal['rotation']['matrix'], dtype=np.float64)
        tr = cal['translation']
        self._t = np.array([tr['x'], tr['y'], tr['z']], dtype=np.float64)
        self.get_logger().info(
            f'Calibration loaded: t=[{self._t[0]:.3f}, {self._t[1]:.3f}, {self._t[2]:.3f}] m'
        )

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

        # ── Publishers ─────────────────────────────────────────────────────
        self._pub_poses   = self.create_publisher(PoseArray,    '/pedestrian_detection/poses',       10)
        self._pub_markers = self.create_publisher(MarkerArray,  '/pedestrian_detection/markers',     10)
        self._pub_debug   = self.create_publisher(Image,        '/pedestrian_detection/debug_image', 5)

        self.get_logger().info(
            f'\n{"=" * 58}\n'
            f'  Pedestrian Detector\n'
            f'  RGB:   {rgb_topic}\n'
            f'  LiDAR: {lidar_topic}\n'
            f'  Model: {model_path}  |  conf={self._conf}\n'
            f'  GPU:   {torch.cuda.get_device_name(0)}\n'
            f'  Frame: {self._lidar_frame}\n'
            f'{"=" * 58}'
        )

    # ── Main callback ──────────────────────────────────────────────────────

    def _cb(self, img_msg: Image, info_msg: CameraInfo, pc_msg: PointCloud2):
        # Camera intrinsics
        K = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(info_msg.d, dtype=np.float64)

        try:
            frame = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion: {e}', throttle_duration_sec=5.0)
            return

        h, w = frame.shape[:2]

        # ── LiDAR → camera projection ──────────────────────────────────────
        pts_lidar = self._pc2_to_xyz(pc_msg)

        # Range filter
        ranges = np.linalg.norm(pts_lidar, axis=1)
        pts_lidar = pts_lidar[ranges < self._max_range]

        # Transform to camera optical frame
        pts_cam = (self._R @ pts_lidar.T).T + self._t

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

        # ── YOLO detection ─────────────────────────────────────────────────
        results = self._model(
            frame,
            conf=self._conf,
            classes=[0],        # person only
            device=self._device,
            verbose=False,
        )

        # ── Per-detection pose extraction ──────────────────────────────────
        poses_3d  = []   # (x, y, z) in velodyne frame
        debug_img = frame.copy()
        stamp     = img_msg.header.stamp

        for r in results:
            if r.boxes is None:
                continue
            boxes = r.boxes.xyxy.cpu().numpy()
            confs = r.boxes.conf.cpu().numpy()

            for (x1, y1, x2, y2), conf in zip(boxes, confs):
                x1, y1, x2, y2 = int(x1), int(y1), int(x2), int(y2)

                # LiDAR points inside this bounding box
                inside = (px >= x1) & (px <= x2) & (py >= y1) & (py <= y2)
                pts_in = pts_lidar[inside]

                if len(pts_in) < self._min_pts:
                    # Draw box grey — no LiDAR hit
                    cv2.rectangle(debug_img, (x1, y1), (x2, y2), (100, 100, 100), 2)
                    cv2.putText(debug_img, 'no LiDAR', (x1, y1 - 6),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (100, 100, 100), 1)
                    continue

                # Median centroid in velodyne frame (robust to outliers)
                cx3d, cy3d, cz3d = np.median(pts_in, axis=0)
                dist = float(np.sqrt(cx3d**2 + cy3d**2))
                poses_3d.append((cx3d, cy3d, cz3d, conf))

                # ── Debug overlay ──────────────────────────────────────────
                cv2.rectangle(debug_img, (x1, y1), (x2, y2), (0, 220, 0), 2)
                label = f'{conf:.2f} | ({cx3d:.1f},{cy3d:.1f},{cz3d:.1f})m  d={dist:.1f}m'
                cv2.putText(debug_img, label, (x1, max(y1 - 6, 10)),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

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
                if len(pts_in) > 0:
                    cp_cam = self._R @ np.array([cx3d, cy3d, cz3d]) + self._t
                    if cp_cam[2] > 0:
                        u = int(K[0, 0] * cp_cam[0] / cp_cam[2] + K[0, 2])
                        v = int(K[1, 1] * cp_cam[1] / cp_cam[2] + K[1, 2])
                        if 0 <= u < w and 0 <= v < h:
                            cv2.drawMarker(debug_img, (u, v), (0, 255, 255),
                                           cv2.MARKER_CROSS, 16, 2)

        # ── Publish ────────────────────────────────────────────────────────
        self._publish_poses(poses_3d, stamp)
        self._publish_markers(poses_3d, stamp)

        # Throttle debug image to save network bandwidth (Jetson → PC)
        now = self.get_clock().now().nanoseconds * 1e-9
        if self._debug_period == 0.0 or (now - self._last_debug_t) >= self._debug_period:
            self._publish_debug(debug_img, stamp)
            self._last_debug_t = now

    # ── Publishers ─────────────────────────────────────────────────────────

    def _publish_poses(self, poses_3d, stamp):
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
        self._pub_poses.publish(msg)

    def _publish_markers(self, poses_3d, stamp):
        msg = MarkerArray()

        # Delete all previous markers first
        del_marker = Marker()
        del_marker.action = Marker.DELETEALL
        msg.markers.append(del_marker)

        for i, (x, y, z, conf) in enumerate(poses_3d):
            # Sphere at centroid
            m = Marker()
            m.header.stamp    = stamp
            m.header.frame_id = self._lidar_frame
            m.ns     = 'pedestrian'
            m.id     = i
            m.type   = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x  = float(x)
            m.pose.position.y  = float(y)
            m.pose.position.z  = float(z)
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.5
            m.color.r = 1.0
            m.color.g = 0.3
            m.color.b = 0.0
            m.color.a = 0.8
            m.lifetime.sec = 1
            msg.markers.append(m)

            # Distance text label
            txt = Marker()
            txt.header.stamp    = stamp
            txt.header.frame_id = self._lidar_frame
            txt.ns     = 'pedestrian_label'
            txt.id     = i
            txt.type   = Marker.TEXT_VIEW_FACING
            txt.action = Marker.ADD
            txt.pose.position.x  = float(x)
            txt.pose.position.y  = float(y)
            txt.pose.position.z  = float(z) + 0.8
            txt.pose.orientation.w = 1.0
            txt.scale.z  = 0.3
            txt.color.r  = 1.0
            txt.color.g  = 1.0
            txt.color.b  = 1.0
            txt.color.a  = 1.0
            dist = float(np.sqrt(x**2 + y**2))
            txt.text     = f'{dist:.1f}m  ({x:.1f},{y:.1f},{z:.1f})'
            txt.lifetime.sec = 1
            msg.markers.append(txt)

        self._pub_markers.publish(msg)

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
    node = PedestrianDetectorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
