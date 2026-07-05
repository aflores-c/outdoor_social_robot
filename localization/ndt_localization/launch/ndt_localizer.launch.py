"""
NDT 3D LiDAR localizer — mirrors the role of amcl_2d_localization but uses
the VLP-32C and a pre-built 3D PCD map instead of a 2D occupancy grid.

Publishes:
    TF:  map → odom            (Nav2 contract, same as AMCL)
    /localization/pose          PoseWithCovarianceStamped
    /localization/map_cloud     PointCloud2 (latched, for RViz)

Requires:
    TF:  odom → base_footprint  (from TIAGo wheel odometry)
    /velodyne_points             PointCloud2 (from VLP-32C)
    /initialpose                 (set via RViz "2D Pose Estimate")

Usage:
    # Minimal — map_path overrides the value in ndt_localizer.yaml
    ros2 launch ndt_localization ndt_localizer.launch.py \\
        map_path:=/home/cas/ndt_map.pcd

    # With known start pose (skips RViz manual step)
    ros2 launch ndt_localization ndt_localizer.launch.py \\
        map_path:=/home/cas/ndt_map.pcd \\
        initial_x:=1.5  initial_y:=0.3  initial_yaw:=0.0
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_share   = get_package_share_directory('ndt_localization')
    params_file = os.path.join(pkg_share, 'config', 'ndt_localizer.yaml')

    return LaunchDescription([

        # ── Launch arguments ─────────────────────────────────────────────────

        DeclareLaunchArgument(
            'map_path',
            default_value=os.path.join(
                get_package_share_directory('fast_lio_robot_bringup'), 'maps', 'ndt_map.pcd'),
            description='Path to the NDT map PCD (output of ndt_map_creator)'
        ),
        DeclareLaunchArgument(
            'use_sim_time', default_value='false',
            description='Use simulation clock'
        ),
        DeclareLaunchArgument(
            'initial_x',   default_value='0.0',
            description='Initial pose X in map frame (metres)'
        ),
        DeclareLaunchArgument(
            'initial_y',   default_value='0.0',
            description='Initial pose Y in map frame (metres)'
        ),
        DeclareLaunchArgument(
            'initial_yaw', default_value='0.0',
            description='Initial pose yaw in map frame (radians)'
        ),

        # ── NDT localizer node ───────────────────────────────────────────────

        Node(
            package='ndt_localization',
            executable='ndt_localizer',
            name='ndt_localizer',
            parameters=[
                params_file,
                {
                    'map_path':     LaunchConfiguration('map_path'),
                    'use_sim_time': LaunchConfiguration('use_sim_time'),
                    'initial_x':    LaunchConfiguration('initial_x'),
                    'initial_y':    LaunchConfiguration('initial_y'),
                    'initial_yaw':  LaunchConfiguration('initial_yaw'),
                },
            ],
            output='screen',
        ),
    ])
