"""
OpenStreetMap GPS visualizer.

Launch:
    ros2 launch osm_gps_visualizer osm_visualize.launch.py

Then open:
    http://localhost:8080

Change port if 8080 is already in use:
    ros2 launch osm_gps_visualizer osm_visualize.launch.py port:=8888
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port_arg = DeclareLaunchArgument(
        'port', default_value='8080',
        description='TCP port for the web server')

    osm_node = Node(
        package='osm_gps_visualizer',
        executable='osm_node',
        name='osm_gps_visualizer',
        output='screen',
        parameters=[{'port': LaunchConfiguration('port')}],
    )

    return LaunchDescription([port_arg, osm_node])
