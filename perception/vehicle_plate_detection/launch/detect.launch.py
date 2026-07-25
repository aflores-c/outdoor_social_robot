"""
Launch vehicle plate detection.

Starts:
  - Intel RealSense D455 camera   (unless launch_camera:=false)
  - plate_detector_node

Published topics:
  /perception/plate_allowed              std_msgs/Bool
  /vehicle_plate_detection/last_plate    std_msgs/String
  /vehicle_plate_detection/debug_image   sensor_msgs/Image

Usage:
    ros2 launch vehicle_plate_detection detect.launch.py

    # Camera already running (e.g. via traffic_object_detection's launch)
    ros2 launch vehicle_plate_detection detect.launch.py launch_camera:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg_detection = get_package_share_directory('vehicle_plate_detection')

    default_config = os.path.join(pkg_detection, 'config', 'plate_detection.yaml')
    default_plates = os.path.join(pkg_detection, 'config', 'registered_plates.yaml')

    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera', default_value='true',
        description='Launch RealSense D455 camera',
    )
    config_arg = DeclareLaunchArgument(
        'config_file', default_value=default_config,
        description='Path to plate_detection.yaml',
    )
    plates_arg = DeclareLaunchArgument(
        'registered_plates_file', default_value=default_plates,
        description='Path to registered_plates.yaml (school allow-list)',
    )
    model_arg = DeclareLaunchArgument(
        'plate_model', default_value='license_plate_yolov8n.pt',
        description='Plate-detector YOLO weights (.pt or .engine)',
    )

    realsense_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        name='realsense2_camera',
        namespace='camera',
        output='screen',
        parameters=[{
            'enable_color': True,
            'enable_depth': False,
            'color_width':  1280,
            'color_height': 720,
            'color_fps':    30,
            'enable_gyro':  False,
            'enable_accel': False,
        }],
        condition=IfCondition(LaunchConfiguration('launch_camera')),
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
        launch_camera_arg,
        config_arg,
        plates_arg,
        model_arg,
        realsense_node,
        detector_node,
    ])
