"""
Launch the wheel-odometry TF broadcaster, as an alternative to
scan_matcher_bringup's VLP-32C-based laser odometry.

IMPORTANT: only one odometry source should have publish_tf enabled at a
time -- check whether mobile_base_controller already broadcasts
odom -> base_footprint itself before setting publish_tf:=true here, and
make sure scan_matcher_bringup isn't also running with publish_tf:=true.

Usage:
    ros2 launch wheel_odometry_bringup wheel_odometry.launch.py

    # Once confirmed no other node publishes odom -> base_footprint:
    ros2 launch wheel_odometry_bringup wheel_odometry.launch.py publish_tf:=true
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('wheel_odometry_bringup')
    default_config = os.path.join(pkg_share, 'config', 'wheel_odometry.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to wheel_odometry.yaml',
    )
    odom_topic_arg = DeclareLaunchArgument(
        'odom_topic', default_value='/mobile_base_controller/odom',
        description='nav_msgs/Odometry topic published by the mobile base controller',
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_tf', default_value='false',
        description='Broadcast odom -> base_footprint from this topic. Leave false '
                     'unless you have confirmed no other node already publishes it.',
    )

    wheel_odom_node = Node(
        package='wheel_odometry_bringup',
        executable='wheel_odom_tf_node',
        name='wheel_odom_tf_node',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {
                'odom_topic': LaunchConfiguration('odom_topic'),
                'publish_tf': LaunchConfiguration('publish_tf'),
            },
        ],
    )

    return LaunchDescription([
        config_arg,
        odom_topic_arg,
        publish_tf_arg,
        wheel_odom_node,
    ])
