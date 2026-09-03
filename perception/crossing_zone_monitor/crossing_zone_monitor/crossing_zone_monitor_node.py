#!/usr/bin/env python3
"""
Crossing-zone occupancy monitor.

Ground-filters the raw Velodyne point cloud, transforms the surviving
points into map_frame, and tests them against a fixed quadrilateral zone
(zone_x/zone_y, 4 corners in map_frame, calibrated per site — same
tf2_echo-based approach used for school_traffic_control's pose_a/pose_b)
covering the vehicle-crossing lane. Publishes a single Bool: True means at
least one obstacle point currently falls inside the zone.

This is a safety signal independent of school_traffic_control's own
camera+lidar vehicle classification (traffic_object_detection): a vehicle
passing close to the robot can leave that camera's field of view (or get
occluded) well before it's actually clear of the crossing lane. This node
doesn't classify anything — it's a plain "is there any obstacle in this
fixed patch of ground" check, meant to gate school_traffic_control's
RETURNING transition alongside (not instead of) the existing
camera-based "vehicle gone" check.

Inputs:
  <pointcloud_topic>  sensor_msgs/PointCloud2  (default /velodyne_points)

Outputs:
  <output_topic>       std_msgs/Bool  (default /perception/crossing_zone_occupied)
"""

import numpy as np

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from tf2_ros import Buffer, ConnectivityException, ExtrapolationException, LookupException, TransformListener

from sensor_msgs.msg import PointCloud2
from std_msgs.msg import Bool


def _quat_to_matrix(x: float, y: float, z: float, w: float) -> np.ndarray:
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w)],
        [2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)],
    ])


def _points_in_polygon(xs: np.ndarray, ys: np.ndarray,
                        poly_x, poly_y) -> np.ndarray:
    """Ray-casting point-in-polygon test, vectorized over point arrays.
    poly_x/poly_y are the zone's corners in order — need not be
    axis-aligned (a real crossing lane usually isn't)."""
    n = len(poly_x)
    inside = np.zeros(len(xs), dtype=bool)
    j = n - 1
    for i in range(n):
        xi, yi = poly_x[i], poly_y[i]
        xj, yj = poly_x[j], poly_y[j]
        crosses = (yi > ys) != (yj > ys)
        x_at_y = (xj - xi) * (ys - yi) / (yj - yi + 1e-12) + xi
        inside ^= crosses & (xs < x_at_y)
        j = i
    return inside


class CrossingZoneMonitorNode(Node):

    def __init__(self):
        super().__init__('crossing_zone_monitor_node')

        # ── Parameters ────────────────────────────────────────────────────
        self.declare_parameter('pointcloud_topic', '/velodyne_points')
        self.declare_parameter('output_topic', '/perception/crossing_zone_occupied')
        self.declare_parameter('lidar_frame', 'velodyne')
        self.declare_parameter('map_frame', 'map')

        # Zone corners in map_frame, in order (need not be axis-aligned).
        # PLACEHOLDER zeros — calibrate per site the same way pose_a/pose_b
        # are (ros2 run tf2_ros tf2_echo map base_link at each corner of
        # the crossing lane) before relying on this in the field; left at
        # zero, the zone is degenerate and will never contain a point.
        self.declare_parameter('zone_x', [0.0, 0.0, 0.0, 0.0])
        self.declare_parameter('zone_y', [0.0, 0.0, 0.0, 0.0])

        # Ground filtering, relative to map_frame after transform — same
        # reasoning/defaults as base_navigation's obstacle_voxel_layer:
        # verify against this robot's actual base_link<->velodyne TF (ros2
        # run tf2_ros tf2_echo base_link velodyne) before trusting these in
        # the field. min_obstacle_height only needs to clear ground-plane
        # return noise, max_obstacle_height ignores anything unrealistically
        # tall for a vehicle.
        self.declare_parameter('min_obstacle_height', 0.10)
        self.declare_parameter('max_obstacle_height', 2.0)

        pointcloud_topic = self.get_parameter('pointcloud_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._lidar_frame = self.get_parameter('lidar_frame').value
        self._map_frame = self.get_parameter('map_frame').value
        self._zone_x = list(self.get_parameter('zone_x').value)
        self._zone_y = list(self.get_parameter('zone_y').value)
        self._min_obstacle_height = float(self.get_parameter('min_obstacle_height').value)
        self._max_obstacle_height = float(self.get_parameter('max_obstacle_height').value)

        if len(self._zone_x) != len(self._zone_y) or len(self._zone_x) < 3:
            self.get_logger().error(
                f'zone_x/zone_y must be equal-length lists of at least 3 corners '
                f'(got {len(self._zone_x)}/{len(self._zone_y)}) — this node will '
                f'never report the zone occupied until fixed.')

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self)

        self._pub_occupied = self.create_publisher(Bool, output_topic, 10)
        self.create_subscription(
            PointCloud2, pointcloud_topic, self._pointcloud_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'crossing_zone_monitor_node ready — {pointcloud_topic} -> {output_topic}, '
            f'zone corners (map_frame): {list(zip(self._zone_x, self._zone_y))}')

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

    def _pointcloud_cb(self, msg: PointCloud2):
        try:
            tf = self._tf_buffer.lookup_transform(self._map_frame, self._lidar_frame, Time())
        except (LookupException, ConnectivityException, ExtrapolationException) as e:
            self.get_logger().warn(
                f'TF lookup {self._map_frame} <- {self._lidar_frame} failed: {e}',
                throttle_duration_sec=5.0)
            return

        pts_lidar = self._pc2_to_xyz(msg)
        if len(pts_lidar) == 0:
            self._publish(False)
            return

        tr = tf.transform.translation
        q = tf.transform.rotation
        R = _quat_to_matrix(q.x, q.y, q.z, q.w)
        t = np.array([tr.x, tr.y, tr.z], dtype=np.float64)
        pts_map = (R @ pts_lidar.T).T + t

        # Ground removal — same height-band approach as base_navigation's
        # obstacle_voxel_layer, applied here in map_frame after transform.
        height_ok = ((pts_map[:, 2] >= self._min_obstacle_height)
                     & (pts_map[:, 2] <= self._max_obstacle_height))
        pts_map = pts_map[height_ok]
        if len(pts_map) == 0:
            self._publish(False)
            return

        inside = _points_in_polygon(pts_map[:, 0], pts_map[:, 1], self._zone_x, self._zone_y)
        self._publish(bool(np.any(inside)))

    def _publish(self, occupied: bool):
        self._pub_occupied.publish(Bool(data=occupied))


def main(args=None):
    rclpy.init(args=args)
    node = CrossingZoneMonitorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
