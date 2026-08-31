#!/usr/bin/env python3
"""
Base-scan close-proximity safety net.

Watches the robot base's own 2D lidar (SICK front+rear, merged and filtered
by PAL's omni_base_laser_sensors, then relayed to /safe_scan by this
package's own launch file) — distinct from the roof-mounted Velodyne
VLP-32C used for vehicle/pedestrian classification — and publishes a plain
Bool: True whenever anything is within proximity_range_m of the base,
False otherwise. No classification, no debouncing (school_traffic_control's
own freshness check + alert cooldown already smooth this downstream) —
just "is something very close right now".

base_scan_proximity.launch.py brings up PAL's omni_base_laser_sensors
(sick_tim drivers + ira_laser_tools merger + pal_laser_filters, not part of
this git workspace) itself and relays its /scan[_front_raw|_rear_raw|_raw]
topics under a safe_ prefix — this node only ever reads the relayed one.

Inputs:
  <scan_topic>              sensor_msgs/msg/LaserScan  (default /safe_scan)

Outputs:
  <close_proximity_topic>   std_msgs/msg/Bool          (default
                             /perception/close_proximity)
"""

import math

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data

from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool


class BaseScanProximityNode(Node):

    def __init__(self):
        super().__init__('base_scan_proximity_node')

        self.declare_parameter('scan_topic', '/safe_scan')
        self.declare_parameter('proximity_range_m', 1.5)
        self.declare_parameter('close_proximity_topic', '/perception/close_proximity')

        scan_topic = self.get_parameter('scan_topic').value
        self._proximity_range = float(self.get_parameter('proximity_range_m').value)
        close_proximity_topic = self.get_parameter('close_proximity_topic').value

        self._pub = self.create_publisher(Bool, close_proximity_topic, 10)
        self.create_subscription(LaserScan, scan_topic, self._scan_cb, qos_profile_sensor_data)

        self.get_logger().info(
            f"base_scan_proximity_node ready — scan_topic='{scan_topic}', "
            f"proximity_range_m={self._proximity_range}")

    def _scan_cb(self, msg: LaserScan):
        close = False
        for r in msg.ranges:
            if math.isnan(r) or math.isinf(r):
                continue
            if r < msg.range_min or r > msg.range_max:
                continue
            if r <= self._proximity_range:
                close = True
                break
        self._pub.publish(Bool(data=close))


def main(args=None):
    rclpy.init(args=args)
    node = BaseScanProximityNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
