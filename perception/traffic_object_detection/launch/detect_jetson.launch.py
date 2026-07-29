"""
Combined pedestrian + vehicle detection launch for Jetson Orin.

Assumes the following are already running externally:
  - RealSense D455 camera  (on this Jetson)
  - Velodyne VLP-32C       (on the remote PC, visible via ROS_DOMAIN_ID)

Starts:
  - lidar_camera static TF  (from calibration result; unless publish_calibrated_tf:=false)
  - traffic_object_detector_node

The detector reads the camera<->LiDAR transform live from TF every frame, not
from a calibration file. The chest D455 isn't otherwise connected to the robot
body in the URDF, so publish_calibrated_tf:=true (default) publishes it here.
For head_front_camera, skip it (publish_calibrated_tf:=false) — that
connection already exists via the robot's own URDF + velodyne_vlp32c_bringup.

Usage:
    ros2 launch traffic_object_detection detect_jetson.launch.py

    # head_front_camera
    ros2 launch traffic_object_detection detect_jetson.launch.py \\
        publish_calibrated_tf:=false config_file:=<path-to>/detection_head_front.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.actions import IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_detection   = get_package_share_directory('traffic_object_detection')
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
        description='Path to lidar_to_camera.yaml (only used if publish_calibrated_tf:=true)',
    )
    publish_tf_arg = DeclareLaunchArgument(
        'publish_calibrated_tf', default_value='true',
        description='Publish camera<->velodyne as a static TF from calibration_file. '
                     'Set false if that connection already exists in the URDF/TF tree '
                     '(e.g. head_front_camera).',
    )
    model_arg = DeclareLaunchArgument(
        'yolo_model', default_value='yolov8n.pt',
        description='YOLO model weights (.pt or .engine for TensorRT)',
    )

    # ── Calibrated static TF (chest D455 only — see module docstring) ──────
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
        condition=IfCondition(LaunchConfiguration('publish_calibrated_tf')),
    )

    # ── Traffic object detector ────────────────────────────────────────────
    detector_node = Node(
        package='traffic_object_detection',
        executable='traffic_object_detector_node',
        name='traffic_object_detector_node',
        output='screen',
        emulate_tty=True,
        parameters=[
            LaunchConfiguration('config_file'),
            {'yolo_model': LaunchConfiguration('yolo_model')},
        ],
    )

    return LaunchDescription([
        config_arg,
        cal_arg,
        publish_tf_arg,
        model_arg,
        publish_tf_launch,
        detector_node,
    ])
