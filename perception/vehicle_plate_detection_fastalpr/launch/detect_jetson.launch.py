"""
Vehicle plate detection (fast-alpr) launch for Jetson Orin.

Assumes the camera is already running on this Jetson (e.g. started by
traffic_object_detection's Jetson launch).

Starts:
  - plate_detector_fastalpr_node

Usage:
    ros2 launch vehicle_plate_detection_fastalpr detect_jetson.launch.py

See also: vehicle_plate_detection/launch/detect_jetson.launch.py — the
original YOLO+EasyOCR pipeline. Only run one of the two at a time if both
publish to the same plate_allowed_topic.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_detection = get_package_share_directory('vehicle_plate_detection_fastalpr')

    default_config = os.path.join(pkg_detection, 'config', 'plate_detection_fastalpr.yaml')
    default_plates = os.path.join(pkg_detection, 'config', 'registered_plates.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to plate_detection_fastalpr.yaml',
    )
    plates_arg = DeclareLaunchArgument(
        'registered_plates_file', default_value=default_plates,
        description='Path to registered_plates.yaml (school allow-list)',
    )

    detector_node = Node(
        package='vehicle_plate_detection_fastalpr',
        executable='plate_detector_fastalpr_node',
        name='plate_detector_fastalpr_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            LaunchConfiguration('registered_plates_file'),
        ],
    )

    return LaunchDescription([
        config_arg,
        plates_arg,
        detector_node,
    ])
