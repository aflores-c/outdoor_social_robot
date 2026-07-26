"""
Launch the outdoor 2D laser-scan-matching odometry node.

Wraps ros2_laser_scan_matcher (vendored from
github.com/AlexKaravaev/ros2_laser_scan_matcher, built on the vendored csm
library — see localization/ros2_laser_scan_matcher and localization/csm)
with this workspace's frame/topic conventions: remaps its "scan"
subscription onto /scan_outdoor (published by velodyne_vlp32c_bringup's
vlp32c_outdoor.launch.py) and publishes the odom -> base_footprint TF that
amcl_2d_localization's mapping/localization launch files require.

Usage:
    ros2 launch scan_matcher_bringup scan_matcher.launch.py

    # Also publish nav_msgs/Odometry for debugging
    ros2 launch scan_matcher_bringup scan_matcher.launch.py \\
        publish_odom:=/scan_matcher/odom

Required inputs:
    <scan_topic>                          sensor_msgs/LaserScan (default /scan_outdoor)
    TF base_footprint -> <laser_frame>     static, from vlp32c_outdoor.launch.py's
                                           torso_lift_link->velodyne transform plus the
                                           robot's own base_footprint->torso_lift_link chain

Published:
    TF odom -> base_footprint
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share = get_package_share_directory('scan_matcher_bringup')
    default_config = os.path.join(pkg_share, 'config', 'scan_matcher.yaml')

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic', default_value='/scan_outdoor',
        description='LaserScan topic to match against (velodyne_laserscan output)',
    )
    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to scan_matcher.yaml',
    )
    publish_odom_arg = DeclareLaunchArgument(
        'publish_odom', default_value='',
        description='Set to a topic name (e.g. /scan_matcher/odom) to also publish '
                     'nav_msgs/Odometry. Empty disables it (TF-only).',
    )

    scan_matcher_node = Node(
        package='ros2_laser_scan_matcher',
        executable='laser_scan_matcher',
        name='laser_scan_matcher',
        output='screen',
        parameters=[
            LaunchConfiguration('config_file'),
            {'publish_odom': LaunchConfiguration('publish_odom')},
        ],
        remappings=[
            ('scan', LaunchConfiguration('scan_topic')),
        ],
    )

    return LaunchDescription([
        scan_topic_arg,
        config_arg,
        publish_odom_arg,
        scan_matcher_node,
    ])
