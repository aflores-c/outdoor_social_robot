import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    pkg = get_package_share_directory('semantic_segmentation')

    # ── Launch arguments ─────────────────────────────────────────────────────

    platform_arg = DeclareLaunchArgument(
        'platform',
        default_value='rtx_desktop',
        description='Hardware preset: "rtx_desktop" or "jetson_orin"'
    )

    params_file_arg = DeclareLaunchArgument(
        'params_file',
        default_value='',
        description='Full path to a custom YAML file (overrides platform preset)'
    )

    # ── Resolve config file ───────────────────────────────────────────────────
    # If params_file is empty the platform preset is used.
    # Users can always override with:
    #   ros2 launch semantic_segmentation segmentation.launch.py \
    #       params_file:=/path/to/my.yaml

    from launch.conditions import IfCondition, UnlessCondition
    from launch.substitutions import PythonExpression

    use_custom = PythonExpression(
        ["'", LaunchConfiguration('params_file'), "' != ''"]
    )

    rtx_config    = os.path.join(pkg, 'config', 'rtx_desktop.yaml')
    jetson_config = os.path.join(pkg, 'config', 'jetson_orin.yaml')

    # Node with custom params_file
    seg_node_custom = Node(
        package='semantic_segmentation',
        executable='segmentation_node',
        name='segmentation_node',
        parameters=[LaunchConfiguration('params_file')],
        output='screen',
        emulate_tty=True,
        condition=IfCondition(use_custom),
    )

    # Node with platform preset
    seg_node_preset = Node(
        package='semantic_segmentation',
        executable='segmentation_node',
        name='segmentation_node',
        parameters=[PythonExpression([
            f'"{rtx_config}" if "',
            LaunchConfiguration('platform'),
            f'" == "rtx_desktop" else "{jetson_config}"',
        ])],
        output='screen',
        emulate_tty=True,
        condition=UnlessCondition(use_custom),
    )

    return LaunchDescription([
        platform_arg,
        params_file_arg,
        seg_node_custom,
        seg_node_preset,
    ])
