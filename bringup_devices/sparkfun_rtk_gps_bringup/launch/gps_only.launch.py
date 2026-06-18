"""
SparkFun ZED-F9P — ublox_gps node only (no RTK/NTRIP).
Use this to verify raw GPS output before enabling RTK corrections.

Launch:
    ros2 launch sparkfun_rtk_gps_bringup gps_only.launch.py
"""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
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
                'enable_rtcm': False,
                'rate': 1.0,
                'nav_rate': 1,
                'dynamic_model': 'portable',
                'tmode3': 0,
            }]
        )
    ])
