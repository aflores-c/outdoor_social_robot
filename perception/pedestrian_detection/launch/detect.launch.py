"""
Launch pedestrian detection + pose estimation.

Starts:
  - Velodyne VLP-32C driver       (unless launch_lidar:=false)
  - Intel RealSense D455 camera   (unless launch_camera:=false)
  - lidar_camera static TF        (from calibration result)
  - pedestrian_detector_node

Published topics:
  /pedestrian_detection/poses        PoseArray   (velodyne frame)
  /pedestrian_detection/markers      MarkerArray (RViz spheres + labels)
  /pedestrian_detection/debug_image  Image       (annotated camera view)

Usage:
    # Full launch (sensors not running yet)
    ros2 launch pedestrian_detection detect.launch.py

    # Sensors already running
    ros2 launch pedestrian_detection detect.launch.py \\
        launch_lidar:=false launch_camera:=false
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_detection = get_package_share_directory('pedestrian_detection')
    pkg_calibration = get_package_share_directory('lidar_camera_calibration')

    default_config = os.path.join(pkg_detection, 'config', 'detection.yaml')
    default_cal    = os.path.join(pkg_calibration, 'results', 'lidar_to_camera.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────
    launch_lidar_arg = DeclareLaunchArgument(
        'launch_lidar', default_value='true',
        description='Launch Velodyne VLP-32C driver',
    )
    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera', default_value='true',
        description='Launch RealSense D455 camera',
    )
    device_ip_arg = DeclareLaunchArgument(
        'device_ip', default_value='10.68.0.55',
        description='Velodyne device IP',
    )
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

    # ── Velodyne VLP-32C ───────────────────────────────────────────────────
    vlp32c_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('velodyne_vlp32c_bringup'),
                'launch', 'vlp32c.launch.py',
            ])
        ]),
        launch_arguments={'device_ip': LaunchConfiguration('device_ip')}.items(),
        condition=IfCondition(LaunchConfiguration('launch_lidar')),
    )

    # ── RealSense D455 ─────────────────────────────────────────────────────
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
        launch_lidar_arg,
        launch_camera_arg,
        device_ip_arg,
        config_arg,
        cal_arg,
        model_arg,
        vlp32c_launch,
        realsense_node,
        publish_tf_launch,
        detector_node,
    ])
