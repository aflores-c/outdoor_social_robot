import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('crossing_zone_monitor')

    return LaunchDescription([

        Node(
            package='crossing_zone_monitor',
            executable='crossing_zone_monitor_node',
            name='crossing_zone_monitor_node',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'crossing_zone_monitor.yaml')],
        ),
    ])
