#!/usr/bin/env python3
"""
GPS pose visualizer for the outdoor robot.

Subscribes to /fix (NavSatFix) and the TF map→base_link transform, then publishes:
  /gps_viz/markers   — accuracy disc + coordinate text + fix-status sphere
  /gps_viz/path      — accumulated robot trajectory in the map frame

Color coding by fix type:
  Green  — RTK FIXED   (accuracy < 5 cm)
  Cyan   — RTK FLOAT   (GBAS but > 5 cm)
  Yellow — DGPS / SBAS
  Orange — Standard GPS
  Red    — No fix
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.duration import Duration
from rclpy.time import Time
from builtin_interfaces.msg import Duration as DurationMsg
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from sensor_msgs.msg import NavSatFix, NavSatStatus
from visualization_msgs.msg import Marker, MarkerArray
import tf2_ros


_RTK_FIXED_THRESH_M = 0.05   # accuracy below this → RTK fixed label


class GpsInfoNode(Node):
    def __init__(self):
        super().__init__('gps_info_node')

        self.declare_parameter('map_frame',   'map')
        self.declare_parameter('base_frame',  'base_link')
        self.declare_parameter('update_hz',   2.0)
        self.declare_parameter('trail_max',   1000)

        self._map_frame  = self.get_parameter('map_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._trail_max  = self.get_parameter('trail_max').value

        self._tf_buf = tf2_ros.Buffer()
        self._tf_lis = tf2_ros.TransformListener(self._tf_buf, self)

        self._latest_fix: NavSatFix = None
        self._path = Path()
        self._path.header.frame_id = self._map_frame

        self.create_subscription(NavSatFix, '/fix', self._fix_cb, 10)

        self._marker_pub = self.create_publisher(MarkerArray, '/gps_viz/markers', 10)
        self._path_pub   = self.create_publisher(Path,        '/gps_viz/path',    10)

        hz = self.get_parameter('update_hz').value
        self.create_timer(1.0 / hz, self._update)

        self.get_logger().info(
            f'GPS Info Node ready — map={self._map_frame}, base={self._base_frame}')

    # ── callbacks ─────────────────────────────────────────────────────────────

    def _fix_cb(self, msg: NavSatFix):
        self._latest_fix = msg

    # ── helpers ───────────────────────────────────────────────────────────────

    def _robot_pos(self):
        """Return robot translation in map frame, or None."""
        try:
            tf = self._tf_buf.lookup_transform(
                self._map_frame, self._base_frame,
                Time(), timeout=Duration(seconds=0.1))
            return tf.transform.translation
        except Exception:
            return None

    def _accuracy_m(self, fix: NavSatFix):
        """1σ horizontal accuracy in metres from covariance, or None."""
        if fix.position_covariance_type == NavSatFix.COVARIANCE_TYPE_UNKNOWN:
            return None
        c = fix.position_covariance
        east  = math.sqrt(max(c[0], 0.0))
        north = math.sqrt(max(c[4], 0.0))
        return max(east, north)

    def _classify(self, fix: NavSatFix, acc):
        """Returns (label, rgba) tuple."""
        s = fix.status.status
        if s == NavSatStatus.STATUS_NO_FIX:
            return 'NO FIX',    (0.85, 0.0,  0.0,  0.9)
        if s == NavSatStatus.STATUS_GBAS_FIX and acc is not None and acc < _RTK_FIXED_THRESH_M:
            return 'RTK FIXED', (0.0,  0.95, 0.2,  0.9)
        if s == NavSatStatus.STATUS_GBAS_FIX:
            return 'RTK FLOAT', (0.0,  0.85, 0.85, 0.85)
        if s == NavSatStatus.STATUS_SBAS_FIX:
            return 'DGPS/SBAS', (1.0,  0.85, 0.0,  0.85)
        return 'GPS FIX',       (1.0,  0.45, 0.0,  0.85)

    def _lifetime(self, sec=2):
        d = DurationMsg()
        d.sec = sec
        return d

    # ── main update ───────────────────────────────────────────────────────────

    def _update(self):
        fix = self._latest_fix
        pos = self._robot_pos()

        if fix is None or pos is None:
            if fix is None:
                self.get_logger().warn('Waiting for /fix …', throttle_duration_sec=5.0)
            if pos is None:
                self.get_logger().warn(
                    f'TF {self._map_frame}→{self._base_frame} not yet available …',
                    throttle_duration_sec=5.0)
            return

        now  = self.get_clock().now().to_msg()
        acc  = self._accuracy_m(fix)
        label, (cr, cg, cb, ca) = self._classify(fix, acc)

        # ── GPS path (accumulate) ──────────────────────────────────────────
        ps = PoseStamped()
        ps.header.stamp       = now
        ps.header.frame_id    = self._map_frame
        ps.pose.position.x    = pos.x
        ps.pose.position.y    = pos.y
        ps.pose.position.z    = pos.z
        ps.pose.orientation.w = 1.0
        self._path.poses.append(ps)
        if len(self._path.poses) > self._trail_max:
            self._path.poses.pop(0)
        self._path.header.stamp = now
        self._path_pub.publish(self._path)

        # ── markers ────────────────────────────────────────────────────────
        markers = MarkerArray()

        # 1. Accuracy disc — flat cylinder on the ground, radius = 1σ accuracy
        radius = max(acc, 0.005) if acc is not None else 3.0
        disc = Marker()
        disc.header.frame_id    = self._map_frame
        disc.header.stamp       = now
        disc.ns                 = 'gps_accuracy_disc'
        disc.id                 = 0
        disc.type               = Marker.CYLINDER
        disc.action             = Marker.ADD
        disc.lifetime           = self._lifetime(3)
        disc.pose.position.x    = pos.x
        disc.pose.position.y    = pos.y
        disc.pose.position.z    = -0.05
        disc.pose.orientation.w = 1.0
        disc.scale.x            = 2.0 * radius   # diameter east
        disc.scale.y            = 2.0 * radius   # diameter north
        disc.scale.z            = 0.03            # 3 cm thick disc
        disc.color.r, disc.color.g, disc.color.b, disc.color.a = cr, cg, cb, ca
        markers.markers.append(disc)

        # 2. Fix-status sphere — coloured ball at robot height + 0.5 m
        sphere = Marker()
        sphere.header.frame_id    = self._map_frame
        sphere.header.stamp       = now
        sphere.ns                 = 'gps_status_sphere'
        sphere.id                 = 1
        sphere.type               = Marker.SPHERE
        sphere.action             = Marker.ADD
        sphere.lifetime           = self._lifetime(3)
        sphere.pose.position.x    = pos.x
        sphere.pose.position.y    = pos.y
        sphere.pose.position.z    = pos.z + 0.5
        sphere.pose.orientation.w = 1.0
        sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.25
        sphere.color.r, sphere.color.g, sphere.color.b, sphere.color.a = cr, cg, cb, 1.0
        markers.markers.append(sphere)

        # 3. Coordinate + accuracy text — floating 2.5 m above robot
        lat_s = f'{abs(fix.latitude):.7f} {"N" if fix.latitude  >= 0 else "S"}'
        lon_s = f'{abs(fix.longitude):.7f} {"E" if fix.longitude >= 0 else "W"}'
        alt_s = f'{fix.altitude:.2f} m'
        if acc is not None:
            if acc < 0.01:
                acc_s = f'{acc*100:.1f} mm (1 sigma)'
            elif acc < 1.0:
                acc_s = f'{acc*100:.1f} cm (1 sigma)'
            else:
                acc_s = f'{acc:.2f} m (1 sigma)'
        else:
            acc_s = 'unknown'

        txt_content = (
            f'FIX : {label}\n'
            f'LAT : {lat_s}\n'
            f'LON : {lon_s}\n'
            f'ALT : {alt_s}\n'
            f'ACC : {acc_s}'
        )
        txt = Marker()
        txt.header.frame_id    = self._map_frame
        txt.header.stamp       = now
        txt.ns                 = 'gps_text'
        txt.id                 = 2
        txt.type               = Marker.TEXT_VIEW_FACING
        txt.action             = Marker.ADD
        txt.lifetime           = self._lifetime(3)
        txt.pose.position.x    = pos.x
        txt.pose.position.y    = pos.y
        txt.pose.position.z    = pos.z + 2.5
        txt.pose.orientation.w = 1.0
        txt.scale.z            = 0.28        # character height in metres
        txt.color.r = txt.color.g = txt.color.b = txt.color.a = 1.0
        txt.text = txt_content
        markers.markers.append(txt)

        self._marker_pub.publish(markers)

        # ── terminal log ──────────────────────────────────────────────────
        self.get_logger().info(
            f'[{label}]  lat={fix.latitude:.7f}  lon={fix.longitude:.7f}  '
            f'alt={fix.altitude:.2f} m  acc={acc_s}',
            throttle_duration_sec=1.0)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(GpsInfoNode())
    rclpy.shutdown()


if __name__ == '__main__':
    main()
