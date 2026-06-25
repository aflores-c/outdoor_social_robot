"""
Validate calibration by projecting LiDAR points onto the camera image.

Starts:
  - Velodyne VLP-32C driver  (unless launch_lidar:=false)
  - Intel RealSense D455 camera  (unless launch_camera:=false)
  - validate_projection_node
  - publish_transform (static TF from calibration result)

View the debug image:
    ros2 run rqt_image_view rqt_image_view
    # select /calibration/debug_image

What to look for:
  GOOD: LiDAR point boundaries line up with the edges of objects in the image.
        Board edges visible in the image should have dense LiDAR dots right on them.
  BAD:  LiDAR dots are offset from visible edges → calibration needs more samples
        or better board orientations.

Usage:
    ros2 launch lidar_camera_calibration validate.launch.py
    ros2 launch lidar_camera_calibration validate.launch.py \\
        result_file:=/path/to/lidar_to_camera.yaml
"""

import os
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('lidar_camera_calibration')
    default_config = os.path.join(pkg_share, 'config', 'calibration.yaml')
    default_result = str(
        Path.home() / '.ros' / 'lidar_camera_calibration' / 'lidar_to_camera.yaml'
    )

    # ── Launch arguments ───────────────────────────────────────────────────────

    launch_lidar_arg = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Launch Velodyne VLP-32C driver. Set false for rosbag replay.',
    )

    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch RealSense D455 camera. Set false for rosbag replay.',
    )

    result_file_arg = DeclareLaunchArgument(
        'result_file',
        default_value=default_result,
        description='Path to lidar_to_camera.yaml produced by estimate_transform',
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to calibration.yaml for validate_projection_node parameters',
    )

    device_ip_arg = DeclareLaunchArgument(
        'device_ip',
        default_value='10.68.0.55',
        description='Velodyne device IP address',
    )

    # ── Velodyne VLP-32C ───────────────────────────────────────────────────────
    vlp32c_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('velodyne_vlp32c_bringup'),
                'launch',
                'vlp32c.launch.py',
            ])
        ]),
        launch_arguments={'device_ip': LaunchConfiguration('device_ip')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_lidar')),
    )

    # ── RealSense D455 ─────────────────────────────────────────────────────────
    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'color_width': 1280,
            'color_height': 720,
            'color_fps': 30,
            'enable_gyro': False,
            'enable_accel': False,
        }],
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    # ── Publish calibrated TF ──────────────────────────────────────────────────
    publish_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_camera_calibration'),
                'launch',
                'publish_transform.launch.py',
            ])
        ]),
        launch_arguments={
            'result_file': LaunchConfiguration('result_file'),
        }.items(),
    )

    # ── Validation node ────────────────────────────────────────────────────────
    validate_node = Node(
        package='lidar_camera_calibration',
        executable='validate_projection_node',
        name='validate_projection_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {'result_file': LaunchConfiguration('result_file')},
        ],
    )

    return LaunchDescription([
        launch_lidar_arg,
        launch_camera_arg,
        result_file_arg,
        config_arg,
        device_ip_arg,
        vlp32c_launch,
        realsense_node,
        publish_tf_launch,
        validate_node,
    ])
