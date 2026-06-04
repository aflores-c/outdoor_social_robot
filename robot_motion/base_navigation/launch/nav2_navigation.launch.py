from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    pkg_dir = get_package_share_directory('base_navigation')

    return LaunchDescription([

        Node(
            package='nav2_planner',
            executable='planner_server',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config/planner_server.yaml')]
        ),

        Node(
            package='nav2_controller',
            executable='controller_server',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config/controller_server.yaml')]
        ),

        Node(
            package='nav2_bt_navigator',
            executable='bt_navigator',
            name='bt_navigator_outdoor',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config/nav2_params.yaml')]
        ),

        Node(
            package='nav2_behaviors',
            executable='behavior_server',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config/nav2_params.yaml')]
        ),

        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_nav',
            output='screen',
            parameters=[os.path.join(pkg_dir, 'config/nav2_params.yaml')]
        ),

        Node(
            package='base_navigation',
            executable='navigate_to_pose_server',
            name='navigate_to_pose_server',
            output='screen',
        ),
    ])
