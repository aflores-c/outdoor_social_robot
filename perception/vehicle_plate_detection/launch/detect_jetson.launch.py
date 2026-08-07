"""
Vehicle plate detection launch for Jetson Orin.

Assumes the RealSense D455 camera is already running on this Jetson
(e.g. started by traffic_object_detection's Jetson launch).

Starts:
  - plate_detector_node

Usage:
    ros2 launch vehicle_plate_detection detect_jetson.launch.py
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_detection = get_package_share_directory('vehicle_plate_detection')

    default_config = os.path.join(pkg_detection, 'config', 'plate_detection.yaml')
    default_plates = os.path.join(pkg_detection, 'config', 'registered_plates.yaml')
    default_model = os.path.join(pkg_detection, 'models', 'license_plate_detector.pt')

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to plate_detection.yaml',
    )
    plates_arg = DeclareLaunchArgument(
        'registered_plates_file', default_value=default_plates,
        description='Path to registered_plates.yaml (school allow-list)',
    )
    model_arg = DeclareLaunchArgument(
        'plate_model', default_value=default_model,
        description='Plate-detector YOLO weights (.pt or .engine)',
    )

    detector_node = Node(
        package='vehicle_plate_detection',
        executable='plate_detector_node',
        name='plate_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            LaunchConfiguration('registered_plates_file'),
            {'plate_model': LaunchConfiguration('plate_model')},
        ],
    )

    return LaunchDescription([
        config_arg,
        plates_arg,
        model_arg,
        detector_node,
    ])
