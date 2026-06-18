"""
GPS + pose visualizer for the outdoor robot.

Launch:
    ros2 launch gps_pose_visualizer visualize.launch.py

For FAST-LIO (world frame is camera_init instead of map):
    ros2 launch gps_pose_visualizer visualize.launch.py map_frame:=camera_init base_frame:=body
"""

import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg = get_package_share_directory('gps_pose_visualizer')
    rviz_cfg = os.path.join(pkg, 'rviz', 'outdoor_robot.rviz')

    map_frame_arg = DeclareLaunchArgument(
        'map_frame', default_value='map',
        description='Fixed world frame (map for LIO-SAM, camera_init for FAST-LIO)')

    base_frame_arg = DeclareLaunchArgument(
        'base_frame', default_value='base_link',
        description='Robot base frame (base_link or body)')

    update_hz_arg = DeclareLaunchArgument(
        'update_hz', default_value='2.0',
        description='GPS marker update rate in Hz')

    gps_info_node = Node(
        package='gps_pose_visualizer',
        executable='gps_info_node',
        name='gps_info_node',
        output='screen',
        parameters=[{
            'map_frame':  LaunchConfiguration('map_frame'),
            'base_frame': LaunchConfiguration('base_frame'),
            'update_hz':  LaunchConfiguration('update_hz'),
            'trail_max':  1000,
        }]
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_cfg],
        output='screen',
    )

    return LaunchDescription([
        map_frame_arg,
        base_frame_arg,
        update_hz_arg,
        gps_info_node,
        rviz_node,
    ])
