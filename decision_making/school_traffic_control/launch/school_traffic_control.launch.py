import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('school_traffic_control')

    return LaunchDescription([

        Node(
            package='school_traffic_control',
            executable='school_traffic_control_node',
            name='school_traffic_control_node',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'school_traffic_control.yaml')],
        ),
    ])
