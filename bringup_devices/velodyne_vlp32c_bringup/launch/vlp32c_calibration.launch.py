from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, RegisterEventHandler, EmitEvent
from launch.substitutions import LaunchConfiguration
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():

    # -------------------------
    # Launch Arguments
    # -------------------------
    device_ip_arg = DeclareLaunchArgument(
        'device_ip',
        default_value='10.68.0.55',
        description='IP address of the Velodyne sensor'
    )

    frame_id_arg = DeclareLaunchArgument(
        'frame_id',
        default_value='velodyne',
        description='Frame id for published pointcloud'
    )

    scan_topic_arg = DeclareLaunchArgument(
        'scan_topic',
        default_value='/scan_outdoor',
        description='Output LaserScan topic name'
    )

    device_ip = LaunchConfiguration('device_ip')
    frame_id = LaunchConfiguration('frame_id')
    scan_topic = LaunchConfiguration('scan_topic')

    # -------------------------
    # Velodyne Driver
    # -------------------------
    driver_share = get_package_share_directory('velodyne_driver')
    driver_params = os.path.join(
        driver_share,
        'config',
        'VLP32C-velodyne_driver_node-params.yaml'
    )

    velodyne_driver_node = Node(
        package='velodyne_driver',
        executable='velodyne_driver_node',
        output='screen',
        parameters=[
            driver_params,
            {'device_ip': device_ip}
        ]
    )

    # -------------------------
    # Velodyne Transform
    # -------------------------
    pointcloud_share = get_package_share_directory('velodyne_pointcloud')

    transform_params = os.path.join(
        pointcloud_share,
        'config',
        'VLP32C-velodyne_transform_node-params.yaml'
    )

    calibration_file = os.path.join(
        pointcloud_share,
        'params',
        'VeloView-VLP-32C.yaml'
    )

    velodyne_transform_node = Node(
        package='velodyne_pointcloud',
        executable='velodyne_transform_node',
        output='screen',
        parameters=[
            transform_params,
            {
                'calibration': calibration_file,
                'frame_id': frame_id
            }
        ]
    )

    # -------------------------
    # Velodyne LaserScan Converter
    # -------------------------
    laserscan_share = get_package_share_directory('velodyne_laserscan')
    laserscan_params = os.path.join(
        laserscan_share,
        'config',
        'default-velodyne_laserscan_node-params.yaml'
    )

    velodyne_laserscan_node = Node(
        package='velodyne_laserscan',
        executable='velodyne_laserscan_node',
        output='screen',
        parameters=[laserscan_params],
        remappings=[
            ('scan', scan_topic)   # <-- THIS IS THE IMPORTANT PART
        ]
    )

    # -------------------------
    # Static Transform Publisher
    # -------------------------
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.36',
            '--y', '-0.00',
            '--z', '0.28',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.00',
            '--frame-id', 'head_front_camera_color_frame',
            '--child-frame-id', frame_id
        ]
    )

    # -------------------------
    # Launch Description
    # -------------------------
    return LaunchDescription([
        device_ip_arg,
        frame_id_arg,
        scan_topic_arg,
        velodyne_driver_node,
        velodyne_transform_node,
        velodyne_laserscan_node,
        static_transform_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])