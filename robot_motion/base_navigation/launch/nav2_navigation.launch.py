import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *args, **kwargs):
    pkg_dir = get_package_share_directory('base_navigation')

    # "diff_drive" (default) constrains MPPI to forward/backward + rotation,
    # matching the robot's actual differential-drive kinematics.
    # "omni" allows lateral (vy) motion in the sampled trajectories.
    controller_profile = LaunchConfiguration('controller_profile').perform(context)
    controller_server_config = os.path.join(
        pkg_dir, 'config', f'controller_server_{controller_profile}.yaml')

    return [
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
            parameters=[controller_server_config]
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
    ]


def generate_launch_description():

    declare_controller_profile = DeclareLaunchArgument(
        'controller_profile',
        default_value='diff_drive',
        choices=['diff_drive', 'omni'],
        description='Controller server motion model profile to load '
                     '(diff_drive: forward/backward only, default; '
                     'omni: allows lateral motion)',
    )

    return LaunchDescription([
        declare_controller_profile,
        OpaqueFunction(function=launch_setup),
    ])
