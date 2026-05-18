import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('semantic_bev')

    # ── Launch arguments ─────────────────────────────────────────────────────

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value=os.path.join(pkg, 'config', 'semantic_bev.yaml'),
        description='Full path to the semantic_bev parameter YAML file'
    )

    use_sim_time_arg = DeclareLaunchArgument(
        'use_sim_time',
        default_value='false',
        description='Use simulation clock'
    )

    rviz_arg = DeclareLaunchArgument(
        'rviz',
        default_value='false',
        description='Launch RViz with the semantic BEV config'
    )

    rviz_cfg_arg = DeclareLaunchArgument(
        'rviz_cfg',
        default_value=os.path.join(pkg, 'rviz', 'semantic_bev.rviz'),
        description='Path to RViz config file'
    )

    # ── Nodes ────────────────────────────────────────────────────────────────

    bev_node = Node(
        package='semantic_bev',
        executable='semantic_bev_node',
        name='semantic_bev_node',
        parameters=[
            LaunchConfiguration('params_file'),
            {'use_sim_time': LaunchConfiguration('use_sim_time')},
        ],
        output='screen',
        emulate_tty=True,
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2_bev',
        arguments=['-d', LaunchConfiguration('rviz_cfg')],
        condition=IfCondition(LaunchConfiguration('rviz')),
    )

    return LaunchDescription([
        params_file_arg,
        use_sim_time_arg,
        rviz_arg,
        rviz_cfg_arg,
        bev_node,
        rviz_node,
    ])
