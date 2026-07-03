import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('xsens_mti_imu_bringup')
    default_config = os.path.join(pkg, 'config', 'xsens_m320.yaml')

    config_arg = DeclareLaunchArgument(
        'config_file',
        default_value=default_config,
        description='Path to xsens parameter YAML',
    )

    imu_node = Node(
        package='bluespace_ai_xsens_mti_driver',
        executable='xsens_mti_node',
        name='xsens_mti_node',
        output='screen',
        emulate_tty=True,
        parameters=[LaunchConfiguration('config_file')],
    )

    base_link_to_imu_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='base_link_to_imu_link_tf',
        arguments=[
            '--x',     '-0.25',
            '--y',     '0.0',
            '--z',     '1.3',
            '--roll',  '0.0',
            '--pitch', '0.0',
            '--yaw',   '3.14159265358979',
            '--frame-id',       'base_link',
            '--child-frame-id', 'imu_link',
        ],
        output='screen'
    )

    return LaunchDescription([
        config_arg,
        imu_node,
        base_link_to_imu_link_tf,
    ])
