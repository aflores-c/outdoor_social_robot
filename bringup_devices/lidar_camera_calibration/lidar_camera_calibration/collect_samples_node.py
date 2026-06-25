#!/usr/bin/env python3
"""
Collect synchronized LiDAR + camera samples for LiDAR-camera extrinsic calibration.

For each sample:
  - Detects a ChArUco board in the color image
    → board plane normal + centroid in camera_color_optical_frame
  - Fits a plane to the ROI-filtered LiDAR point cloud via RANSAC
    → board plane normal + centroid in velodyne frame
  - Saves the plane pair to a JSON file for offline estimation

Capture samples:
    ros2 service call /calibration/capture std_srvs/srv/Trigger

Or enable auto_capture_interval_s in calibration.yaml to capture on a timer.

Minimum recommended: 6 samples with clearly different board orientations.
"""

import json
import threading
from pathlib import Path
from typing import Optional, Tuple

import cv2
import cv2.aruco as aruco
import numpy as np
import rclpy
from cv_bridge import CvBridge
from message_filters import ApproximateTimeSynchronizer, Subscriber
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo, Image, PointCloud2
from std_srvs.srv import Trigger


class CollectSamplesNode(Node):

    def __init__(self):
        super().__init__('collect_samples_node')

        # ── Parameters ────────────────────────────────────────────────────────
        self.declare_parameter('image_topic', '/camera/color/image_raw')
        self.declare_parameter('camera_info_topic', '/camera/color/camera_info')
        self.declare_parameter('lidar_topic', '/velodyne_points')
        self.declare_parameter('board.squares_x', 5)
        self.declare_parameter('board.squares_y', 7)
        self.declare_parameter('board.square_size_m', 0.10)
        self.declare_parameter('board.marker_size_m', 0.075)
        self.declare_parameter('board.aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('lidar_roi.x_min', 0.5)
        self.declare_parameter('lidar_roi.x_max', 8.0)
        self.declare_parameter('lidar_roi.y_min', -2.5)
        self.declare_parameter('lidar_roi.y_max', 2.5)
        self.declare_parameter('lidar_roi.z_min', -0.3)
        self.declare_parameter('lidar_roi.z_max', 2.5)
        self.declare_parameter('ransac.distance_threshold_m', 0.02)
        self.declare_parameter('ransac.max_iterations', 1000)
        self.declare_parameter('ransac.min_inliers', 80)
        self.declare_parameter('sync_slop_s', 0.05)
        self.declare_parameter('min_samples', 6)
        self.declare_parameter('auto_capture_interval_s', -1.0)
        self.declare_parameter('output_file', '')

        # ── ChArUco board ──────────────────────────────────────────────────────
        dict_name = self.get_parameter('board.aruco_dict').value
        aruco_dict_id = getattr(aruco, dict_name, aruco.DICT_4X4_50)
        self._dictionary = aruco.Dictionary_get(aruco_dict_id)
        self._board = aruco.CharucoBoard_create(
            squaresX=self.get_parameter('board.squares_x').value,
            squaresY=self.get_parameter('board.squares_y').value,
            squareLength=self.get_parameter('board.square_size_m').value,
            markerLength=self.get_parameter('board.marker_size_m').value,
            dictionary=self._dictionary,
        )
        self._det_params = aruco.DetectorParameters_create()
        # Tuned for large markers at distance
        self._det_params.adaptiveThreshWinSizeMin = 3
        self._det_params.adaptiveThreshWinSizeMax = 53
        self._det_params.adaptiveThreshWinSizeStep = 10
        self._det_params.minMarkerPerimeterRate = 0.02

        self._bridge = CvBridge()

        # ── State ──────────────────────────────────────────────────────────────
        self._samples: list = []
        self._last_valid: Optional[dict] = None
        self._last_valid_time: float = 0.0
        self._lock = threading.Lock()

        # ── Output path ────────────────────────────────────────────────────────
        out = self.get_parameter('output_file').value
        if not out:
            out = str(Path.home() / '.ros' / 'lidar_camera_calibration' / 'samples.json')
        self._output_file = out
        Path(self._output_file).parent.mkdir(parents=True, exist_ok=True)

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
            queue_size=10,
            slop=slop,
        )
        self._sync.registerCallback(self._sync_cb)

        # ── Publishers ──────────────────────────────────────────────────────────
        self._pub_debug = self.create_publisher(Image, '/calibration/debug_image', 10)

        # ── Capture service ─────────────────────────────────────────────────────
        self._capture_srv = self.create_service(
            Trigger, '/calibration/capture', self._capture_cb
        )
        self._save_srv = self.create_service(
            Trigger, '/calibration/save', self._save_cb
        )

        # ── Auto-capture timer ──────────────────────────────────────────────────
        interval = self.get_parameter('auto_capture_interval_s').value
        if interval > 0:
            self._auto_timer = self.create_timer(interval, self._auto_capture)
            self.get_logger().info(f'Auto-capture enabled every {interval:.1f} s')

        self.get_logger().info(
            f'\n{"=" * 62}\n'
            f'  Collect Samples Node\n'
            f'  Listening: {image_topic}\n'
            f'              {lidar_topic}\n'
            f'  Capture:   ros2 service call /calibration/capture std_srvs/srv/Trigger\n'
            f'  Debug img: ros2 run rqt_image_view rqt_image_view\n'
            f'             → topic: /calibration/debug_image\n'
            f'  Output:    {self._output_file}\n'
            f'{"=" * 62}'
        )

    # ── Service callbacks ──────────────────────────────────────────────────────

    def _capture_cb(self, _request, response: Trigger.Response):
        with self._lock:
            if self._last_valid is not None:
                self._samples.append(dict(self._last_valid))
                n = len(self._samples)
                self._save_samples()
                nv = self._last_valid
                response.success = True
                response.message = (
                    f'Sample {n} captured. '
                    f'n_cam=[{nv["n_cam"][0]:.3f},{nv["n_cam"][1]:.3f},{nv["n_cam"][2]:.3f}] '
                    f'n_lidar=[{nv["n_lidar"][0]:.3f},{nv["n_lidar"][1]:.3f},{nv["n_lidar"][2]:.3f}]'
                )
                self.get_logger().info(f'[✓] {response.message}')
            else:
                response.success = False
                response.message = 'No valid detection. Board not found or LiDAR plane not fitted.'
                self.get_logger().warn(f'[✗] {response.message}')
        return response

    def _save_cb(self, _request, response: Trigger.Response):
        n = len(self._samples)
        if n == 0:
            response.success = False
            response.message = 'No samples to save.'
        else:
            self._save_samples()
            response.success = True
            response.message = f'Saved {n} samples to {self._output_file}'
            self.get_logger().info(response.message)
        return response

    def _auto_capture(self):
        now = self.get_clock().now().nanoseconds * 1e-9
        with self._lock:
            age = now - self._last_valid_time
            if self._last_valid is not None and age < 2.0:
                self._samples.append(dict(self._last_valid))
                n = len(self._samples)
                self._save_samples()
                self.get_logger().info(f'[auto] Sample {n} captured.')
            elif self._last_valid is not None and age >= 2.0:
                self.get_logger().info(
                    f'[auto] Skipped — last valid detection was {age:.1f}s ago (board not visible)',
                    throttle_duration_sec=5.0,
                )

    # ── Synchronized callback ──────────────────────────────────────────────────

    def _sync_cb(self, img_msg: Image, info_msg: CameraInfo, pc_msg: PointCloud2):
        K = np.array(info_msg.k).reshape(3, 3)
        D = np.array(info_msg.d)

        try:
            img_bgr = self._bridge.imgmsg_to_cv2(img_msg, 'bgr8')
        except Exception as e:
            self.get_logger().warn(f'Image conversion: {e}', throttle_duration_sec=5.0)
            return

        # Camera detection
        n_cam, c_cam, overlay = self._detect_charuco(img_bgr, K, D)

        # LiDAR plane
        pts = self._pc2_to_xyz(pc_msg)
        pts_roi = self._apply_roi(pts)
        n_lidar, c_lidar, _ = self._fit_plane_ransac(pts_roi)

        # Build debug image
        debug = img_bgr.copy()
        cam_ok = n_cam is not None
        lid_ok = n_lidar is not None

        if cam_ok and overlay is not None:
            charuco_corners, charuco_ids, rvec, tvec, _ = overlay
            aruco.drawDetectedCornersCharuco(debug, charuco_corners, charuco_ids)
            cv2.drawFrameAxes(debug, K, D, rvec, tvec, 0.1)

        n_samples = len(self._samples)
        min_s = self.get_parameter('min_samples').value

        lines = [
            (f'BOARD:  {"OK  (" + str(len(overlay[0])) + " corners)" if (cam_ok and overlay) else "NOT DETECTED"}',
             (0, 220, 0) if cam_ok else (0, 80, 255)),
            (f'LIDAR:  {"OK" if lid_ok else "PLANE NOT FOUND  (check ROI / min_inliers)"}',
             (0, 220, 0) if lid_ok else (0, 80, 255)),
            (f'Samples: {n_samples}  (need {min_s} for calibration)',
             (255, 220, 0)),
            ('Capture: ros2 service call /calibration/capture std_srvs/srv/Trigger',
             (200, 200, 200)),
        ]
        for i, (txt, col) in enumerate(lines):
            y = 30 + i * 30
            cv2.putText(debug, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 3)
            cv2.putText(debug, txt, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2)

        try:
            self._pub_debug.publish(self._bridge.cv2_to_imgmsg(debug, 'bgr8'))
        except Exception:
            pass

        # Update last valid detection
        if cam_ok and lid_ok:
            with self._lock:
                self._last_valid = {
                    'n_cam': n_cam.tolist(),
                    'c_cam': c_cam.tolist(),
                    'n_lidar': n_lidar.tolist(),
                    'c_lidar': c_lidar.tolist(),
                }
                self._last_valid_time = self.get_clock().now().nanoseconds * 1e-9

    # ── ChArUco detection ──────────────────────────────────────────────────────

    def _detect_charuco(
        self, img_bgr: np.ndarray, K: np.ndarray, D: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[tuple]]:
        gray = cv2.cvtColor(img_bgr, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = aruco.detectMarkers(gray, self._dictionary, parameters=self._det_params)

        if ids is None or len(ids) < 4:
            return None, None, None

        retval, charuco_corners, charuco_ids = aruco.interpolateCornersCharuco(
            corners, ids, gray, self._board
        )
        if not retval or charuco_corners is None or len(charuco_corners) < 6:
            return None, None, None

        valid, rvec, tvec = aruco.estimatePoseCharucoBoard(
            charuco_corners, charuco_ids, self._board, K, D, None, None
        )
        if not valid:
            return None, None, None

        R_board, _ = cv2.Rodrigues(rvec)
        # Board +Z axis = front face normal in camera frame
        n_cam = R_board @ np.array([0.0, 0.0, 1.0])
        c_cam = tvec.flatten().astype(np.float64)

        # Orient toward camera: in optical frame camera is at origin,
        # board is at positive Z, so the board normal pointing toward camera
        # should have negative Z component.
        if n_cam[2] > 0:
            n_cam = -n_cam
        n_cam /= np.linalg.norm(n_cam)

        return n_cam, c_cam, (charuco_corners, charuco_ids, rvec, tvec, R_board)

    # ── LiDAR utilities ────────────────────────────────────────────────────────

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

    def _apply_roi(self, pts: np.ndarray) -> np.ndarray:
        p = self.get_parameter
        mask = (
            (pts[:, 0] >= p('lidar_roi.x_min').value) &
            (pts[:, 0] <= p('lidar_roi.x_max').value) &
            (pts[:, 1] >= p('lidar_roi.y_min').value) &
            (pts[:, 1] <= p('lidar_roi.y_max').value) &
            (pts[:, 2] >= p('lidar_roi.z_min').value) &
            (pts[:, 2] <= p('lidar_roi.z_max').value)
        )
        return pts[mask]

    def _fit_plane_ransac(
        self, pts: np.ndarray
    ) -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
        dist_thresh = self.get_parameter('ransac.distance_threshold_m').value
        max_iter = self.get_parameter('ransac.max_iterations').value
        min_inliers = self.get_parameter('ransac.min_inliers').value

        if len(pts) < 10:
            return None, None, None

        n_pts = len(pts)
        best_count = 0
        best_mask = None
        rng = np.random.default_rng()

        for _ in range(max_iter):
            idx = rng.choice(n_pts, 3, replace=False)
            p0, p1, p2 = pts[idx]
            v1, v2 = p1 - p0, p2 - p0
            n = np.cross(v1, v2)
            norm_n = np.linalg.norm(n)
            if norm_n < 1e-9:
                continue
            n /= norm_n
            dists = np.abs(pts @ n - np.dot(n, p0))
            mask = dists < dist_thresh
            count = mask.sum()
            if count > best_count:
                best_count = count
                best_mask = mask
            if count > 0.85 * n_pts:
                break  # early exit when clearly dominant

        if best_mask is None or best_count < min_inliers:
            return None, None, None

        # Refine normal via PCA on inliers
        inlier_pts = pts[best_mask]
        centroid = inlier_pts.mean(axis=0)
        _, _, Vt = np.linalg.svd(inlier_pts - centroid, full_matrices=False)
        n_refined = Vt[-1]  # eigenvector of smallest singular value = plane normal

        # Orient toward LiDAR origin (sensor at [0,0,0])
        toward_sensor = -centroid / (np.linalg.norm(centroid) + 1e-9)
        if np.dot(n_refined, toward_sensor) < 0:
            n_refined = -n_refined
        n_refined /= np.linalg.norm(n_refined)

        return n_refined, centroid, best_mask

    # ── Persistence ────────────────────────────────────────────────────────────

    def _save_samples(self):
        try:
            with open(self._output_file, 'w') as f:
                json.dump({'samples': self._samples}, f, indent=2)
        except Exception as e:
            self.get_logger().error(f'Failed to save samples: {e}')

    def destroy_node(self):
        if self._samples:
            self._save_samples()
            n = len(self._samples)
            min_s = self.get_parameter('min_samples').value
            self.get_logger().info(f'Saved {n} samples → {self._output_file}')
            if n < min_s:
                self.get_logger().warn(
                    f'Only {n}/{min_s} recommended samples. '
                    'Calibration may be inaccurate — collect more with varied board orientations.'
                )
            self.get_logger().info(
                f'Next step:\n'
                f'  ros2 run lidar_camera_calibration estimate_transform'
            )
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CollectSamplesNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
