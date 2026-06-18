"""
Slam-toolbox online async mapping.

Drives async_slam_toolbox_node to build a 2-D occupancy map in real time.

When done mapping, save with:
    ros2 run nav2_map_server map_saver_cli -f <output_path>/<map_name>

The saved .pgm + .yaml can be dropped into map/ to be used by amcl_localization.launch.py.

Continue a previous mapping session (serialized map):
    ros2 launch amcl_2d_localization mapping.launch.py \
        load_map:=true  map_name:=my_previous_map

Examples:
    # Start fresh map
    ros2 launch amcl_2d_localization mapping.launch.py

    # Continue a serialised session (map/ folder must contain my_map.data + my_map.posegraph)
    ros2 launch amcl_2d_localization mapping.launch.py \
        load_map:=true  map_name:=my_map

Required inputs:
    /scan_outdoor  (sensor_msgs/LaserScan)
    TF: odom → base_footprint

Published:
    /map           (nav_msgs/OccupancyGrid)
    TF: map → odom
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import (
    LaunchConfiguration,
    PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('amcl_2d_localization')
    slam_config = os.path.join(pkg_share, 'config', 'slam_toolbox.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='Name (no extension) of a serialized slam_toolbox map in map/ '
                    'to resume from. Only used when load_map:=true.'
    )

    load_map_arg = DeclareLaunchArgument(
        'load_map',
        default_value='false',
        description='Set true to resume a previous mapping session from map/<map_name>'
    )

    # ── Serialized map path: <pkg>/map/<map_name>  (no extension — slam_toolbox adds them) ──

    serialized_map = PathJoinSubstitution([
        FindPackageShare('amcl_2d_localization'),
        'map',
        LaunchConfiguration('map_name'),
    ])

    # ── slam_toolbox node: fresh start ───────────────────────────────────

    slam_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(
            # Launch when load_map is false
            # We use a small trick: spawn the "new map" node only when load_map=false
            # (the "resume" variant below handles load_map=true)
            _bool_not(LaunchConfiguration('load_map'))
        ),
        parameters=[
            slam_config,
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
    )

    # ── slam_toolbox node: resume from serialized map ────────────────────

    slam_resume_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        output='screen',
        condition=IfCondition(LaunchConfiguration('load_map')),
        parameters=[
            slam_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'map_file_name': serialized_map,
                'map_start_at_dock': True,
            },
        ],
    )

    return LaunchDescription([
        use_sim_time_arg,
        map_name_arg,
        load_map_arg,
        slam_node,
        slam_resume_node,
    ])


def _bool_not(substitution):
    """Return a substitution that is 'true' when the input evaluates to 'false'."""
    from launch.substitutions import PythonExpression
    return PythonExpression(["'true' if '", substitution, "' == 'false' else 'false'"])
