import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    pkg_dir = get_package_share_directory('benchmark_logging')
    config = os.path.join(pkg_dir, 'config', 'benchmark_logging.yaml')

    return LaunchDescription([

        Node(
            package='benchmark_logging',
            executable='trial_manager_node',
            name='trial_manager_node',
            output='screen',
            parameters=[config],
        ),

        Node(
            package='benchmark_logging',
            executable='data_logger_node',
            name='data_logger_node',
            output='screen',
            parameters=[config],
        ),
    ])
