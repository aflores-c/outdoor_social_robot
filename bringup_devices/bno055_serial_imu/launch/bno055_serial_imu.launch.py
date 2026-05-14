from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():

    imu_node = Node(
        package='bno055_serial_imu',
        executable='serial_imu_node',
        name='bno055_serial_imu_node',
        output='screen',

        parameters=[
            {
                'port': '/dev/ttyACM0',
                'baudrate': 115200,
                'frame_id': 'imu_link',
                'topic_name': '/imu/data'
            }
        ]
    )

    return LaunchDescription([
        imu_node
    ])