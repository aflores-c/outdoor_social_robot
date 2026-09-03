"""
Crossing-zone visualizer for RViz2.

Launch:
    ros2 launch crossing_zone_visualizer visualize.launch.py

Add a MarkerArray display in RViz2 subscribed to /crossing_zone_viz/markers
(Fixed Frame = map) to see it — this launch file does not start RViz2
itself, since it's meant to be added onto an already-running session
(e.g. alongside gps_pose_visualizer's) rather than opening a second one.
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package='crossing_zone_visualizer',
            executable='crossing_zone_viz_node',
            name='crossing_zone_viz_node',
            output='screen',
        ),
    ])
