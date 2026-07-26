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
        description='Output LaserScan topic name (range_max-clamped, see scan_range_clamp_node)'
    )

    max_range_arg = DeclareLaunchArgument(
        'max_range',
        default_value='60.0',
        description='Clamp applied to the published scan_topic range_max. velodyne_laserscan_node '
                     'hardcodes range_max to the VLP-32C spec (200 m), which overflows '
                     "slam_toolbox/Karto's correlation grid at sensor registration and crashes it "
                     '("Mapper FATAL ERROR - unable to get pointer in probability search"). '
                     '60.0 matches amcl_2d_localization/config/slam_toolbox.yaml max_laser_range '
                     '-- keep the two in sync.'
    )

    device_ip = LaunchConfiguration('device_ip')
    frame_id = LaunchConfiguration('frame_id')
    scan_topic = LaunchConfiguration('scan_topic')
    max_range = LaunchConfiguration('max_range')

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
            {
                'device_ip': device_ip,
                'rpm': 600.0,
            }
        ]
    )

    # -------------------------
    # Velodyne Transform
    # -------------------------
    pointcloud_share = get_package_share_directory('velodyne_pointcloud')
    bringup_share = get_package_share_directory('velodyne_vlp32c_bringup')

    transform_params = os.path.join(
        pointcloud_share,
        'config',
        'VLP32C-velodyne_transform_node-params.yaml'
    )

    transform_override = os.path.join(
        bringup_share,
        'config',
        'velodyne_transform_override.yaml'
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
            transform_override,
            {
                'calibration': calibration_file,
                'frame_id': frame_id,
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
            ('scan', 'scan_outdoor_raw')   # unclamped range_max — see scan_range_clamp_node below
        ]
    )

    # -------------------------
    # Range clamp (velodyne_laserscan_node hardcodes range_max to 200 m,
    # which crashes slam_toolbox/Karto — see max_range_arg above)
    # -------------------------
    scan_range_clamp_node = Node(
        package='velodyne_vlp32c_bringup',
        executable='scan_range_clamp_node',
        output='screen',
        parameters=[{
            'input_topic': 'scan_outdoor_raw',
            'output_topic': scan_topic,
            'max_range': max_range,
        }]
    )

    # -------------------------
    # Static Transform Publisher
    # -------------------------
    static_transform_node = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        arguments=[
            '--x', '-0.28',
            '--y', '0.0',
            '--z', '0.60',
            '--roll', '0.0',
            '--pitch', '0.0',
            '--yaw', '0.0',
            '--frame-id', 'torso_lift_link',
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
        max_range_arg,
        velodyne_driver_node,
        velodyne_transform_node,
        velodyne_laserscan_node,
        scan_range_clamp_node,
        static_transform_node,
        RegisterEventHandler(
            OnProcessExit(
                target_action=velodyne_driver_node,
                on_exit=[EmitEvent(event=Shutdown())],
            )
        ),
    ])