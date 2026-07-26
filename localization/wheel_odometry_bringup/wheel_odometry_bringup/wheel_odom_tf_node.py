#!/usr/bin/env python3
"""
Broadcast odom -> base_footprint from the mobile base controller's wheel
odometry, as an alternative odometry source to scan_matcher_bringup's
VLP-32C-based laser odometry.

IMPORTANT: only one odometry source should ever have publish_tf enabled at
a time. Many ros2_control diff_drive_controllers (mobile_base_controller
included) already broadcast this exact TF themselves when their own
enable_odom_tf parameter is on -- check that before enabling publish_tf
here, or you'll end up with two nodes fighting over the same
odom -> base_footprint edge (which looks exactly like the jumpy/unstable
transform seen with two simultaneous odometry publishers).
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from nav_msgs.msg import Odometry
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster


class WheelOdomTfNode(Node):

    def __init__(self):
        super().__init__('wheel_odom_tf_node')

        self.declare_parameter('odom_topic', '/mobile_base_controller/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')
        # Off by default -- see module docstring. Flip on only once you've
        # confirmed mobile_base_controller isn't already broadcasting this TF.
        self.declare_parameter('publish_tf', False)

        odom_topic = self.get_parameter('odom_topic').value
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        self._publish_tf = bool(self.get_parameter('publish_tf').value)

        self._broadcaster = TransformBroadcaster(self)
        self.create_subscription(Odometry, odom_topic, self._cb, qos_profile_sensor_data)

        self.get_logger().info(
            f'wheel_odom_tf_node: {odom_topic} -> TF {self._odom_frame} -> {self._base_frame} '
            f'(publish_tf={self._publish_tf})'
        )
        if not self._publish_tf:
            self.get_logger().warn(
                'publish_tf is false -- subscribing but NOT broadcasting TF. '
                'Set publish_tf:=true once you have confirmed no other node '
                'is already publishing odom -> base_footprint.'
            )

    def _cb(self, msg: Odometry):
        if not self._publish_tf:
            return

        t = TransformStamped()
        t.header.stamp = msg.header.stamp
        t.header.frame_id = self._odom_frame
        t.child_frame_id = self._base_frame
        t.transform.translation.x = msg.pose.pose.position.x
        t.transform.translation.y = msg.pose.pose.position.y
        t.transform.translation.z = msg.pose.pose.position.z
        t.transform.rotation = msg.pose.pose.orientation

        self._broadcaster.sendTransform(t)


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomTfNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
