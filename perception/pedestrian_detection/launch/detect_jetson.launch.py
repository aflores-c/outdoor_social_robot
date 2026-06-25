"""
Pedestrian detection launch for Jetson Orin.

Assumes the following are already running externally:
  - RealSense D455 camera  (on this Jetson)
  - Velodyne VLP-32C       (on the remote PC, visible via ROS_DOMAIN_ID)

Starts:
  - lidar_camera static TF  (from calibration result in lidar_camera_calibration pkg)
  - pedestrian_detector_node

Usage:
    ros2 launch pedestrian_detection detect_jetson.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_detection   = get_package_share_directory('pedestrian_detection')
    pkg_calibration = get_package_share_directory('lidar_camera_calibration')

    default_config = os.path.join(pkg_detection,   'config', 'detection.yaml')
    default_cal    = os.path.join(pkg_calibration, 'results', 'lidar_to_camera.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────
    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to detection.yaml',
    )
    cal_arg = DeclareLaunchArgument(
        'calibration_file', default_value=default_cal,
        description='Path to lidar_to_camera.yaml',
    )
    model_arg = DeclareLaunchArgument(
        'yolo_model', default_value='yolov8n.pt',
        description='YOLO model weights (.pt or .engine for TensorRT)',
    )

    # ── Calibrated static TF ───────────────────────────────────────────────
    publish_tf_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('lidar_camera_calibration'),
                'launch', 'publish_transform.launch.py',
            ])
        ]),
        launch_arguments={
            'result_file': LaunchConfiguration('calibration_file'),
        }.items(),
    )

    # ── Pedestrian detector ────────────────────────────────────────────────
    detector_node = Node(
        package='pedestrian_detection',
        executable='pedestrian_detector_node',
        name='pedestrian_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {'calibration_file': LaunchConfiguration('calibration_file'),
             'yolo_model':       LaunchConfiguration('yolo_model')},
        ],
    )

    return LaunchDescription([
        config_arg,
        cal_arg,
        model_arg,
        publish_tf_launch,
        detector_node,
    ])
