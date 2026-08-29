import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('robot_audio')

    return LaunchDescription([

        Node(
            package='robot_audio',
            executable='robot_audio_node',
            name='robot_audio_node',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config', 'robot_audio.yaml')],
        ),
    ])
