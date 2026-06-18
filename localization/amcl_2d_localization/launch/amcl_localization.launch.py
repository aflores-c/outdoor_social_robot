"""
2D AMCL localization — map_server + amcl + lifecycle_manager.

The map is loaded from the package's own map/ folder.
Pass map_name to select which map file to use (without .yaml extension).

Examples:
    # Use the default map  →  map/map.yaml
    ros2 launch amcl_2d_localization amcl_localization.launch.py

    # Use a different map  →  map/outdoor_lab.yaml
    ros2 launch amcl_2d_localization amcl_localization.launch.py map_name:=outdoor_lab

    # Override initial pose at launch time
    ros2 launch amcl_2d_localization amcl_localization.launch.py \
        map_name:=building_a  initial_x:=3.0  initial_y:=1.5  initial_yaw:=1.57

Required inputs:
    /scan_outdoor  (sensor_msgs/LaserScan)
    TF: odom → base_footprint

Published:
    /map           (nav_msgs/OccupancyGrid)
    /amcl_pose     (geometry_msgs/PoseWithCovarianceStamped)
    /particle_cloud(nav2_msgs/ParticleCloud)
    TF: map → odom
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    pkg_share = get_package_share_directory('amcl_2d_localization')
    amcl_config = os.path.join(pkg_share, 'config', 'amcl.yaml')

    # ── Launch arguments ──────────────────────────────────────────────────

    map_name_arg = DeclareLaunchArgument(
        'map_name',
        default_value='map',
        description='Name of the map file (without .yaml) inside the package map/ folder. '
                    'Example: map_name:=outdoor_lab  loads  map/outdoor_lab.yaml'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    initial_x_arg = DeclareLaunchArgument(
        'initial_x',   default_value='0.0',
        description='Initial pose X (metres, map frame)')

    initial_y_arg = DeclareLaunchArgument(
        'initial_y',   default_value='0.0',
        description='Initial pose Y (metres, map frame)')

    initial_yaw_arg = DeclareLaunchArgument(
        'initial_yaw', default_value='0.0',
        description='Initial pose yaw (radians, map frame)')

    # ── Map file path: <pkg>/map/<map_name>.yaml ──────────────────────────

    map_file = PathJoinSubstitution([
        FindPackageShare('amcl_2d_localization'),
        'map',
        [LaunchConfiguration('map_name'), '.yaml']
    ])

    # ── Nodes ─────────────────────────────────────────────────────────────

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'yaml_filename': map_file,
        }]
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[
            amcl_config,
            {
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'initial_pose.x':   LaunchConfiguration('initial_x'),
                'initial_pose.y':   LaunchConfiguration('initial_y'),
                'initial_pose.yaw': LaunchConfiguration('initial_yaw'),
            }
        ]
    )

    # Lifecycle manager activates map_server then amcl (order matters)
    lifecycle_manager_node = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': LaunchConfiguration('use_sim_time'),
            'autostart': True,
            'node_names': ['map_server', 'amcl'],
        }]
    )

    return LaunchDescription([
        map_name_arg,
        use_sim_time_arg,
        initial_x_arg,
        initial_y_arg,
        initial_yaw_arg,
        map_server_node,
        amcl_node,
        lifecycle_manager_node,
    ])
