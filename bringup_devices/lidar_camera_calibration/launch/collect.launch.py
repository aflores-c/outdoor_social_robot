"""
Launch the LiDAR-camera calibration sample collection session.

Starts:
  - Velodyne VLP-32C driver  (unless launch_lidar:=false)
  - Intel RealSense D455 camera  (unless launch_camera:=false)
  - collect_samples_node

To capture a sample (hold board steady, then call):
    ros2 service call /calibration/capture std_srvs/srv/Trigger

To replay from a rosbag instead of live sensors:
    ros2 launch lidar_camera_calibration collect.launch.py \\
        launch_lidar:=false launch_camera:=false
    # in another terminal:
    ros2 bag play <bag_file>

After collecting ≥ 6 samples, run:
    ros2 run lidar_camera_calibration estimate_transform
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

    pkg_share = get_package_share_directory('lidar_camera_calibration')
    default_config = os.path.join(pkg_share, 'config', 'calibration.yaml')

    # ── Launch arguments ───────────────────────────────────────────────────────

    launch_lidar_arg = DeclareLaunchArgument(
        'launch_lidar',
        default_value='true',
        description='Launch Velodyne VLP-32C driver. Set false when replaying a rosbag.',
    )

    launch_camera_arg = DeclareLaunchArgument(
        'launch_camera',
        default_value='true',
        description='Launch RealSense D455 camera. Set false when replaying a rosbag.',
    )

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to calibration.yaml parameter file',
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
            'enable_depth': False,       # depth not needed during collection
            'color_width': 1280,
            'color_height': 720,
            'color_fps': 30,
            'enable_gyro': False,
            'enable_accel': False,
        }],
        condition=IfCondition(LaunchConfiguration('launch_camera')),
    )

    # ── Collect samples node ───────────────────────────────────────────────────
    collect_node = Node(
        package='lidar_camera_calibration',
        executable='collect_samples_node',
        name='collect_samples_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
    )

    return LaunchDescription([
        launch_lidar_arg,
        launch_camera_arg,
        config_arg,
        device_ip_arg,
        vlp32c_launch,
        realsense_node,
        collect_node,
    ])
