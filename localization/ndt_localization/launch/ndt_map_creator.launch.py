"""
Pre-process a raw FAST-LIO PCD map into a compact NDT localization map.
Run this ONCE after building a new map with FAST-LIO.

Usage:
    ros2 launch ndt_localization ndt_map_creator.launch.py \\
        input_path:=/home/cas/fast_lio_map.pcd \\
        output_path:=/home/cas/ndt_map.pcd

Optional:
    leaf_size:=0.2          # voxel downsample resolution in metres (default 0.2)
    remove_outliers:=true   # statistical outlier removal (default true)
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    return LaunchDescription([

        DeclareLaunchArgument(
            'input_path',
            description='Path to the raw FAST-LIO PCD map (e.g. /home/cas/fast_lio_map.pcd)'
        ),
        DeclareLaunchArgument(
            'output_path',
            description='Where to save the processed NDT map (e.g. /home/cas/ndt_map.pcd)'
        ),
        DeclareLaunchArgument(
            'leaf_size', default_value='0.2',
            description='VoxelGrid leaf size in metres'
        ),
        DeclareLaunchArgument(
            'remove_outliers', default_value='true',
            description='Run StatisticalOutlierRemoval after downsampling'
        ),

        Node(
            package='ndt_localization',
            executable='ndt_map_creator',
            name='ndt_map_creator',
            parameters=[{
                'input_path':      LaunchConfiguration('input_path'),
                'output_path':     LaunchConfiguration('output_path'),
                'leaf_size':       LaunchConfiguration('leaf_size'),
                'remove_outliers': LaunchConfiguration('remove_outliers'),
            }],
            output='screen',
        ),
    ])
