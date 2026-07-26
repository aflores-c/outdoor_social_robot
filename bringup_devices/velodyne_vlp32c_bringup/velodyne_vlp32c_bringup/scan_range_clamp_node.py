#!/usr/bin/env python3
"""
Clamp the VLP-32C-derived LaserScan's advertised range_max down to a value
slam_toolbox/Karto can actually handle.

velodyne_laserscan_node publishes range_max fixed at the sensor's
theoretical spec (200.0 m for the VLP-32C) — it has no parameter to
override this. slam_toolbox's Karto mapper sizes its internal correlation
grid from a scan's range_max *at sensor registration time*, independent of
the mapper's own max_laser_range parameter (which only clips points during
later scan matching). 200 m at typical 0.05 m map resolution overflows
that grid and crashes with:
    "Mapper FATAL ERROR - unable to get pointer in probability search"

This node re-publishes the scan with range_max clamped to a sane value
(and any individual reading beyond it, or NaN, replaced with inf), so
downstream consumers (slam_toolbox, AMCL, the scan matcher) never see the
inflated range.
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan


class ScanRangeClampNode(Node):

    def __init__(self):
        super().__init__('scan_range_clamp_node')

        self.declare_parameter('input_topic', 'scan_outdoor_raw')
        self.declare_parameter('output_topic', '/scan_outdoor')
        self.declare_parameter('max_range', 60.0)

        input_topic = self.get_parameter('input_topic').value
        output_topic = self.get_parameter('output_topic').value
        self._max_range = float(self.get_parameter('max_range').value)

        self._pub = self.create_publisher(LaserScan, output_topic, qos_profile_sensor_data)
        self.create_subscription(LaserScan, input_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'Clamping {input_topic} -> {output_topic}  (range_max={self._max_range} m)'
        )

    def _cb(self, msg: LaserScan):
        msg.range_max = min(msg.range_max, self._max_range)
        msg.ranges = [
            r if (r == r and r <= self._max_range) else float('inf')
            for r in msg.ranges
        ]
        self._pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = ScanRangeClampNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
