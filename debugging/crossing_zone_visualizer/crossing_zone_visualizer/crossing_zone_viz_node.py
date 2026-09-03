#!/usr/bin/env python3
"""
RViz2 visualizer for crossing_zone_monitor's fixed crossing-lane zone.

Reads zone_x/zone_y (and map_frame) directly from crossing_zone_monitor's
own installed config yaml — not duplicated params here — so this can never
silently drift out of sync with what crossing_zone_monitor is actually
checking against. Publishes:
  <markers_topic>  visualization_msgs/MarkerArray (default /crossing_zone_viz/markers)
    - outline (LINE_STRIP): the zone perimeter
    - fill (TRIANGLE_LIST): a semi-transparent fan-triangulated fill
    - status (TEXT_VIEW_FACING): live occupancy state, floating above the zone

Colored by the live signal school_traffic_control actually gates on:
  Red    — occupied (obstacle currently in the zone)
  Green  — confirmed clear (fresh 'not occupied' reading)
  Yellow — unknown (no fresh reading — matches _crossing_zone_clear's
           fail-safe: stale/missing is never shown as clear)
"""

import os

import rclpy
from rclpy.node import Node
from ament_index_python.packages import get_package_share_directory
import yaml

from std_msgs.msg import Bool
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker, MarkerArray

_DEFAULT_CONFIG_PKG = 'crossing_zone_monitor'
_DEFAULT_CONFIG_REL_PATH = os.path.join('config', 'crossing_zone_monitor.yaml')
_DEFAULT_NODE_KEY = 'crossing_zone_monitor_node'

_COLOR_OCCUPIED = (0.9, 0.1, 0.1)
_COLOR_CLEAR = (0.1, 0.9, 0.2)
_COLOR_UNKNOWN = (0.95, 0.85, 0.0)


class CrossingZoneVizNode(Node):

    def __init__(self):
        super().__init__('crossing_zone_viz_node')

        default_config_path = os.path.join(
            get_package_share_directory(_DEFAULT_CONFIG_PKG), _DEFAULT_CONFIG_REL_PATH)
        self.declare_parameter('zone_config_path', default_config_path)
        self.declare_parameter('occupied_topic', '/perception/crossing_zone_occupied')
        self.declare_parameter('markers_topic', '/crossing_zone_viz/markers')
        self.declare_parameter('update_hz', 2.0)
        # Matches school_traffic_control's message_timeout_s convention —
        # a reading older than this is treated as unknown, not clear.
        self.declare_parameter('stale_timeout_s', 1.0)

        zone_config_path = self.get_parameter('zone_config_path').value
        occupied_topic = self.get_parameter('occupied_topic').value
        markers_topic = self.get_parameter('markers_topic').value
        update_hz = float(self.get_parameter('update_hz').value)
        self._stale_timeout = float(self.get_parameter('stale_timeout_s').value)

        self._map_frame, self._zone_x, self._zone_y = self._load_zone(zone_config_path)
        self._centroid = (sum(self._zone_x) / len(self._zone_x),
                           sum(self._zone_y) / len(self._zone_y))

        self._occupied = None  # None = never received
        self._occupied_stamp = None

        self.create_subscription(Bool, occupied_topic, self._on_occupied, 10)
        self._pub_markers = self.create_publisher(MarkerArray, markers_topic, 10)
        self.create_timer(1.0 / update_hz, self._publish_markers)

        self.get_logger().info(
            f'crossing_zone_viz_node ready — zone loaded from {zone_config_path} '
            f'({len(self._zone_x)} corners, frame={self._map_frame}), '
            f'watching {occupied_topic} -> {markers_topic}')

    def _load_zone(self, config_path: str):
        with open(config_path) as f:
            params = yaml.safe_load(f)[_DEFAULT_NODE_KEY]['ros__parameters']
        zone_x = list(params['zone_x'])
        zone_y = list(params['zone_y'])
        map_frame = params.get('map_frame', 'map')
        if len(zone_x) != len(zone_y) or len(zone_x) < 3:
            self.get_logger().error(
                f'{config_path}: zone_x/zone_y must be equal-length lists of at '
                f'least 3 corners (got {len(zone_x)}/{len(zone_y)}) — nothing will '
                f'be drawn correctly.')
        return map_frame, zone_x, zone_y

    def _on_occupied(self, msg: Bool):
        self._occupied = bool(msg.data)
        self._occupied_stamp = self.get_clock().now()

    def _current_color(self):
        if self._occupied_stamp is None:
            return _COLOR_UNKNOWN, 'UNKNOWN (no reading yet)'
        age_s = (self.get_clock().now() - self._occupied_stamp).nanoseconds * 1e-9
        if age_s > self._stale_timeout:
            return _COLOR_UNKNOWN, f'UNKNOWN (stale, {age_s:.1f}s old)'
        if self._occupied:
            return _COLOR_OCCUPIED, 'OCCUPIED'
        return _COLOR_CLEAR, 'CLEAR'

    def _publish_markers(self):
        now = self.get_clock().now().to_msg()
        (r, g, b), status_text = self._current_color()
        n = len(self._zone_x)
        points = [Point(x=self._zone_x[i], y=self._zone_y[i], z=0.05) for i in range(n)]

        outline = Marker()
        outline.header.frame_id = self._map_frame
        outline.header.stamp = now
        outline.ns = 'crossing_zone_outline'
        outline.id = 0
        outline.type = Marker.LINE_STRIP
        outline.action = Marker.ADD
        outline.pose.orientation.w = 1.0
        outline.scale.x = 0.06  # line width
        outline.color.r, outline.color.g, outline.color.b, outline.color.a = r, g, b, 0.9
        outline.points = points + [points[0]]  # close the loop

        fill = Marker()
        fill.header.frame_id = self._map_frame
        fill.header.stamp = now
        fill.ns = 'crossing_zone_fill'
        fill.id = 1
        fill.type = Marker.TRIANGLE_LIST
        fill.action = Marker.ADD
        fill.pose.orientation.w = 1.0
        fill.scale.x = fill.scale.y = fill.scale.z = 1.0
        fill.color.r, fill.color.g, fill.color.b, fill.color.a = r, g, b, 0.25
        # Fan triangulation from the centroid — correct for a convex
        # polygon (the calibrated rectangle), a reasonable approximation
        # for a mildly non-convex one.
        center = Point(x=self._centroid[0], y=self._centroid[1], z=0.04)
        for i in range(n):
            fill.points.extend([center, points[i], points[(i + 1) % n]])

        status = Marker()
        status.header.frame_id = self._map_frame
        status.header.stamp = now
        status.ns = 'crossing_zone_status'
        status.id = 2
        status.type = Marker.TEXT_VIEW_FACING
        status.action = Marker.ADD
        status.pose.position.x = self._centroid[0]
        status.pose.position.y = self._centroid[1]
        status.pose.position.z = 1.0
        status.pose.orientation.w = 1.0
        status.scale.z = 0.3
        status.color.r = status.color.g = status.color.b = status.color.a = 1.0
        status.text = f'CROSSING ZONE: {status_text}'

        self._pub_markers.publish(MarkerArray(markers=[outline, fill, status]))


def main(args=None):
    rclpy.init(args=args)
    node = CrossingZoneVizNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
