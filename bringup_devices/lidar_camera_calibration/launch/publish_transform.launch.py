"""
Publish the calibrated LiDAR → camera static TF.

Reads the YAML produced by estimate_transform and launches a static_transform_publisher
with the calibrated values.

Add this launch file to your robot bringup to activate the calibrated transform.

Usage:
    # Use default result path (~/.ros/lidar_camera_calibration/lidar_to_camera.yaml)
    ros2 launch lidar_camera_calibration publish_transform.launch.py

    # Point to a specific result file
    ros2 launch lidar_camera_calibration publish_transform.launch.py \\
        result_file:=/path/to/lidar_to_camera.yaml

Published TF:  camera_color_optical_frame → velodyne
"""

import os
from pathlib import Path

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _load_and_publish(context, *args, **kwargs):
    result_file = LaunchConfiguration('result_file').perform(context)
    result_path = Path(result_file)

    if not result_path.exists():
        raise FileNotFoundError(
            f'Calibration result not found: {result_path}\n'
            'Run estimate_transform first:\n'
            '  ros2 run lidar_camera_calibration estimate_transform'
        )

    with open(result_path) as f:
        cal = yaml.safe_load(f)['lidar_to_camera']

    parent = cal['parent_frame']
    child = cal['child_frame']
    t = cal['translation']
    q = cal['rotation']['quaternion']
    resid = cal.get('residuals', {}).get('mean_angular_deg', float('nan'))
    n_samples = cal.get('n_samples', '?')

    # Print summary to launch terminal
    print(
        f'\n[publish_transform] Loading calibration from {result_path}\n'
        f'  parent: {parent}\n'
        f'  child:  {child}\n'
        f'  t = [{t["x"]:.6f}, {t["y"]:.6f}, {t["z"]:.6f}] m\n'
        f'  q = [x={q["x"]:.6f}, y={q["y"]:.6f}, z={q["z"]:.6f}, w={q["w"]:.6f}]\n'
        f'  calibrated from {n_samples} samples, mean angular residual: {resid:.3f}°\n'
    )

    node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='lidar_camera_tf',
        output='screen',
        arguments=[
            '--x',   str(t['x']),
            '--y',   str(t['y']),
            '--z',   str(t['z']),
            '--qx',  str(q['x']),
            '--qy',  str(q['y']),
            '--qz',  str(q['z']),
            '--qw',  str(q['w']),
            '--frame-id',       parent,
            '--child-frame-id', child,
        ],
    )
    return [node]


def generate_launch_description():
    pkg_results = os.path.join(
        get_package_share_directory('lidar_camera_calibration'),
        'results',
        'lidar_to_camera.yaml',
    )

    result_file_arg = DeclareLaunchArgument(
        'result_file',
        default_value=pkg_results,
        description='Path to lidar_to_camera.yaml produced by estimate_transform',
    )

    return LaunchDescription([
        result_file_arg,
        OpaqueFunction(function=_load_and_publish),
    ])
