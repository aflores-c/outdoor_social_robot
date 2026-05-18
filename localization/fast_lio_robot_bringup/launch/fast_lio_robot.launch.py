import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():

    bringup_share = get_package_share_directory('fast_lio_robot_bringup')
    fast_lio_share = get_package_share_directory('fast_lio')

    # ── Launch arguments ──────────────────────────────────────────────────────

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(bringup_share, 'config', 'fast_lio_velodyne32.yaml'),
        description='Full path to the FAST-LIO parameter YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation (Gazebo) clock if true'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz for visualisation'
    )

    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=os.path.join(fast_lio_share, 'rviz', 'fastlio.rviz'),
        description='Path to RViz configuration file'
    )

    # ── Nodes ─────────────────────────────────────────────────────────────────

    fast_lio_node = Node(
        package='fast_lio',
        executable='fastlio_mapping',
        name='fastlio_mapping',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')}
        ],
        output='screen'
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        condition=IfCondition(LaunchConfiguration('rviz'))
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        rviz_arg,
        rviz_cfg_arg,
        fast_lio_node,
        rviz_node,
    ])
