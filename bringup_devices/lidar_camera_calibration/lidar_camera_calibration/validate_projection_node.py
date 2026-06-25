#!/usr/bin/env python3
"""
Validate LiDAR-camera extrinsic calibration by projecting LiDAR points onto the color image.

Reads the calibration result YAML, subscribes to image + LiDAR, and publishes a debug
image with LiDAR points projected onto the color frame, colour-coded by depth.

Good calibration: board edges in the image should exactly align with the projected
LiDAR point boundaries.  Misalignment → re-run collection with more samples.

View output:
    ros2 run rqt_image_view rqt_image_view
    # select /calibration/debug_image

    # or in RViz2 add Image display → /calibration/debug_image
"""

from pathlib import Path
from typing import Optional

import cv2
import numpy as np
import rclpy
import yaml
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2


class ValidateProjectionNode(Node):

    def __init__(self):
        super().__init__('validate_projection_node')

        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('result_file', '')
        self.declare_parameter('sync_slop_s', 0.10)
        self.declare_parameter('max_range_m', 20.0)
        self.declare_parameter('point_radius', 2)

        # ── Load calibration ───────────────────────────────────────────────────
        result_file = self.get_parameter('result_file').value
        if not result_file:
            result_file = str(
                Path.home() / '.ros' / 'lidar_camera_calibration' / 'lidar_to_camera.yaml'
            )
        result_path = Path(result_file)
        if not result_path.exists():
            self.get_logger().error(
                f'Calibration result not found: {result_path}\n'
                'Run estimate_transform first:\n'
                '  ros2 run lidar_camera_calibration estimate_transform'
            )
            raise FileNotFoundError(str(result_path))

        with open(result_path) as f:
            cal = yaml.safe_load(f)['lidar_to_camera']

        self._R = np.array(cal['rotation']['matrix'], dtype=np.float64)
        tr = cal['translation']
        self._t = np.array([tr['x'], tr['y'], tr['z']], dtype=np.float64)

        residual = cal.get('residuals', {}).get('mean_angular_deg', float('nan'))
        self.get_logger().info(
            f'Calibration loaded from {result_path}\n'
            f'  t=[{self._t[0]:.4f}, {self._t[1]:.4f}, {self._t[2]:.4f}] m\n'
            f'  mean angular residual: {residual:.3f}°'
        )

        self._bridge = CvBridge()

        # ── Subscribers + sync ─────────────────────────────────────────────────
        image_topic = self.get_parameter('image_topic').value
        info_topic = self.get_parameter('camera_info_topic').value
        lidar_topic = self.get_parameter('lidar_topic').value
        slop = self.get_parameter('sync_slop_s').value

        self._sub_image = Subscriber(self, Image, image_topic)
        self._sub_info = Subscriber(self, CameraInfo, info_topic)
        self._sub_lidar = Subscriber(self, PointCloud2, lidar_topic)
        self._sync = ApproximateTimeSynchronizer(
            [self._sub_image, self._sub_info, self._sub_lidar],
            queue_size=5,
            slop=slop,
        )
        self._sync.registerCallback(self._sync_cb)

        self._pub = self.create_publisher(Image, '/calibration/debug_image', 5)

        self.get_logger().info(
            'Validate projection ready.\n'
            '  View: ros2 run rqt_image_view rqt_image_view\n'
            '        → topic: /calibration/debug_image\n'
            '  What to look for: LiDAR point edges should align with board edges in the image.\n'
            '  Blue = near, Red = far.'
        )

    # ── Sync callback ──────────────────────────────────────────────────────────

    def _sync_cb(self, img_msg: Image, info_msg: CameraInfo, pc_msg: PointCloud2):
        K = np.array(info_msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(info_msg.d, dtype=np.float64)

        try:
            img = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8').copy()
        except Exception as e:
            self.get_logger().warn(f'Image conversion: {e}', throttle_duration_sec=5.0)
            return

        pts = self._pc2_to_xyz(pc_msg)
        if len(pts) == 0:
            return

        # Range filter
        max_r = self.get_parameter('max_range_m').value
        pts = pts[np.linalg.norm(pts, axis=1) < max_r]
        if len(pts) == 0:
            return

        # Transform LiDAR → camera frame
        pts_cam = (self._R @ pts.T).T + self._t

        # Keep only points in front of camera
        front = pts_cam[:, 2] > 0.05
        pts_cam = pts_cam[front]
        if len(pts_cam) == 0:
            return

        # Project to pixel coordinates
        h, w = img.shape[:2]
        fx, fy = K[0, 0], K[1, 1]
        cx, cy = K[0, 2], K[1, 2]

        # Undistort projection (approximate: apply distortion ignored for thin lens)
        # For a proper projection with D, use cv2.projectPoints:
        if D is not None and len(D) > 0 and np.any(D != 0):
            obj_pts = pts_cam.reshape(-1, 1, 3).astype(np.float32)
            img_pts, _ = cv2.projectPoints(obj_pts, np.zeros(3), np.zeros(3), K.astype(np.float32), D.astype(np.float32))
            px = img_pts[:, 0, 0].astype(np.float32)
            py = img_pts[:, 0, 1].astype(np.float32)
        else:
            px = (fx * pts_cam[:, 0] / pts_cam[:, 2] + cx).astype(np.float32)
            py = (fy * pts_cam[:, 1] / pts_cam[:, 2] + cy).astype(np.float32)

        depths = pts_cam[:, 2]

        # Keep in-image points
        in_img = (px >= 0) & (px < w) & (py >= 0) & (py < h)
        px = px[in_img].astype(np.int32)
        py = py[in_img].astype(np.int32)
        depths = depths[in_img]

        if len(px) == 0:
            return

        # Colour by depth: blue=near, red=far
        d_min, d_max = float(depths.min()), float(depths.max())
        t_norm = np.clip((depths - d_min) / (d_max - d_min + 1e-6), 0.0, 1.0)
        radius = self.get_parameter('point_radius').value

        for i in range(len(px)):
            ti = float(t_norm[i])
            b = int(255 * (1.0 - ti))
            r = int(255 * ti)
            cv2.circle(img, (px[i], py[i]), radius, (b, 0, r), -1)

        # HUD overlay
        info = (
            f'Projected: {len(px)} pts | '
            f'Range: {d_min:.1f}–{d_max:.1f} m | '
            f'Blue=near  Red=far'
        )
        cv2.putText(img, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
        cv2.putText(img, info, (10, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 2)

        hint = 'Check: LiDAR board edges should align with image board edges'
        cv2.putText(img, hint, (10, img.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 3)
        cv2.putText(img, hint, (10, img.shape[0] - 12),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (200, 220, 255), 2)

        try:
            self._pub.publish(self._bridge.cv2_to_imgmsg(img, 'bgr8'))
        except Exception:
            pass

    # ── PointCloud2 → xyz ──────────────────────────────────────────────────────

    def _pc2_to_xyz(self, msg: PointCloud2) -> np.ndarray:
        off = {f.name: f.offset for f in msg.fields}
        n = msg.width * msg.height
        step = msg.point_step
        buf = np.frombuffer(bytes(msg.data), dtype=np.uint8).reshape(n, step)
        ox, oy, oz = off['x'], off['y'], off['z']
        xs = buf[:, ox:ox + 4].copy().view(np.float32).reshape(-1)
        ys = buf[:, oy:oy + 4].copy().view(np.float32).reshape(-1)
        zs = buf[:, oz:oz + 4].copy().view(np.float32).reshape(-1)
        pts = np.column_stack([xs, ys, zs]).astype(np.float64)
        return pts[np.isfinite(pts).all(axis=1)]


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ValidateProjectionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except FileNotFoundError:
        pass
    finally:
        try:
            rclpy.shutdown()
        except Exception:
            pass


if __name__ == '__main__':
    main()
