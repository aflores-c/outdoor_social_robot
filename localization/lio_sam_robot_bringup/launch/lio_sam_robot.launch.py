import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

from launch_ros.actions import Node


def generate_launch_description():

    bringup_share = get_package_share_directory('lio_sam_robot_bringup')

    default_params_file = os.path.join(
        bringup_share,
        'config',
        'lio_sam_velodyne32.yaml'
    )

    params_file = LaunchConfiguration('params_file')

    params_arg = DeclareLaunchArgument(
        'params_file',
        default_value=default_params_file,
        description='Path to custom LIO-SAM parameters file'
    )

    imu_preintegration = Node(
        package='lio_sam',
        executable='lio_sam_imuPreintegration',
        name='lio_sam_imuPreintegration',
        parameters=[params_file],
        output='screen'
    )

    image_projection = Node(
        package='lio_sam',
        executable='lio_sam_imageProjection',
        name='lio_sam_imageProjection',
        parameters=[params_file],
        output='screen'
    )

    feature_extraction = Node(
        package='lio_sam',
        executable='lio_sam_featureExtraction',
        name='lio_sam_featureExtraction',
        parameters=[params_file],
        output='screen'
    )

    map_optimization = Node(
        package='lio_sam',
        executable='lio_sam_mapOptimization',
        name='lio_sam_mapOptimization',
        parameters=[params_file],
        output='screen'
    )

    return LaunchDescription([
        params_arg,
        imu_preintegration,
        image_projection,
        feature_extraction,
        map_optimization
    ])