from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0'
    )

    imu_node = Node(
        package='bno055_serial_imu',
        executable='serial_imu_node',
        name='bno055_serial_imu_node',
        output='screen',

        parameters=[
            {
                'port': LaunchConfiguration('port'),
                'baudrate': 115200,
                'frame_id': 'imu_link',
                'topic_name': '/imu/data'
            }
        ]
    )

    # Static TF: base_link -> imu_link
    # Replace xyz/rpy with your real mounting values
    imu_static_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='imu_static_tf',

        arguments=[
            '0.0', '0.0', '0.24',   # x y z (meters)
            '0.0', '0.0', '0.0',    # roll pitch yaw (radians)
            'base_link',
            'imu_link'
        ],

        output='screen'
    )

    return LaunchDescription([
        port_arg,
        imu_static_tf,
        imu_node
    ])