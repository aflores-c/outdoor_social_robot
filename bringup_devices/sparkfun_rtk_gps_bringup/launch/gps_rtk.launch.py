"""
SparkFun ZED-F9P RTK GPS bringup — ublox_gps + NTRIP client (SAPOS BW).

Launch:
    ros2 launch sparkfun_rtk_gps_bringup gps_rtk.launch.py

Before launching, fill in your HEPS credentials in:
    config/ntrip_credentials.yaml

Topic flow:
    ublox_gps_node  →  /fix  (NavSatFix)
    ntrip_client   ←  /fix  (sends rover position to NTRIP caster for VRS)
    ntrip_client    →  /rtcm (rtcm_msgs/Message)
    ublox_gps_node ←  /rtcm (applies RTK corrections)
"""

import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_share = get_package_share_directory('sparkfun_rtk_gps_bringup')
    ntrip_config = os.path.join(pkg_share, 'config', 'ntrip_credentials.yaml')

    ublox_node = Node(
        package='ublox_gps',
        executable='ublox_gps_node',
        name='ublox_gps',
        output='screen',
        parameters=[{
            'device': '/dev/ttyACM0',
            'frame_id': 'gps',
            'baudrate': 115200,
            'ublox_topic_diagnostics': False,
            'enable_raw_data': True,
            'enable_sbas': True,
            'enable_ppp': False,
            'enable_rtcm': True,
            'rate': 1.0,
            'nav_rate': 1,
            'dynamic_model': 'portable',
            'tmode3': 0,
        }]
    )

    # ntrip_client subscribes to /fix (NavSatFix) and sends NMEA GGA to the
    # NTRIP caster automatically — no separate converter node needed.
    ntrip_node = Node(
        package='ntrip_client',
        executable='ntrip_ros.py',
        name='ntrip_client',
        output='screen',
        parameters=[ntrip_config],
    )

    return LaunchDescription([
        ublox_node,
        ntrip_node,
    ])
