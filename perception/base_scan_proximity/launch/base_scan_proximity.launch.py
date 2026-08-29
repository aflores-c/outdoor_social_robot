import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('base_scan_proximity')

    return LaunchDescription([

        Node(
            package='base_scan_proximity',
            executable='base_scan_proximity_node',
            name='base_scan_proximity_node',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'base_scan_proximity.yaml')],
        ),
    ])
