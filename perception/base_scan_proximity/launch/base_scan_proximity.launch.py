import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# omni_base_laser_sensors (PAL, not part of this git workspace) publishes its
# own front/rear/merged/filtered scans under these fixed names -- see
# laser_sick-571.launch.py's underlying sick_laser_cfg/ira_laser_tools/
# pal_laser_filters configs. relay each one under a safe_-prefixed alias
# rather than editing PAL's own launch/config in place, so other consumers
# on the robot that may still expect the original names (nav2's own costmap,
# if PAL's default config uses them) keep working unchanged.
#
# This same launch also activates PAL's direct_laser_odometry (dlo_ros)
# composable node, which by default broadcasts its own odom -> base TF from
# the merged SICK scan -- this collides with scan_matcher_bringup's own
# odom TF (from the outdoor Velodyne, longer range, actually outdoors).
# Disabled on this robot via a PAL user-config override at
# /home/pal/.pal/config/90_disable_dlo_odom.yaml (dlo.enable_publish_odom_tf:
# false) -- not part of this git workspace, see get_pal_configuration()'s
# ~/.pal/config/ override mechanism in launch_pal. direct_laser_odometry
# still runs (still merges/filters scans for base_scan_proximity), it just
# no longer publishes odometry. Re-apply this file if the robot is ever
# re-imaged/reset.
_LASER_SCAN_RELAYS = [
    ('/scan_front_raw', '/safe_scan_front_raw'),
    ('/scan_rear_raw', '/safe_scan_rear_raw'),
    ('/scan_raw', '/safe_scan_raw'),
    ('/scan', '/safe_scan'),
]


def generate_launch_description():

    pkg_dir = get_package_share_directory('base_scan_proximity')

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/safe_scan',
        description="Which relayed scan base_scan_proximity_node reads. Default "
                     '/safe_scan is front+rear merged+filtered. Use '
                     '/safe_scan_front_raw or /safe_scan_rear_raw for a single '
                     'laser only (e.g. front laser only, unmerged, unfiltered), '
                     'or /safe_scan_raw for merged-but-unfiltered.'
    )
    scan_topic = LaunchConfiguration('scan_topic')

    omni_base_laser_sensors = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('omni_base_laser_sensors'),
                'launch', 'laser_sick-571.launch.py')
        )
    )

    relay_nodes = [
        Node(
            package='topic_tools',
            executable='relay',
            name=f'safe_scan_relay_{i}',
            arguments=[src, dst],
        )
        for i, (src, dst) in enumerate(_LASER_SCAN_RELAYS)
    ]

    return LaunchDescription([
        scan_topic_arg,
        omni_base_laser_sensors,
        *relay_nodes,

        Node(
            package='base_scan_proximity',
            executable='base_scan_proximity_node',
            name='base_scan_proximity_node',
            output='screen',
            parameters=[
                os.path.join(pkg_dir, 'config', 'base_scan_proximity.yaml'),
                {'scan_topic': scan_topic},
            ],
        ),
    ])
