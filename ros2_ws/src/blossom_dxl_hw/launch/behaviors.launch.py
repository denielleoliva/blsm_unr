from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('blossom_dxl_hw')

    # Defaults (override via CLI)
    default_behavior_yaml   = os.path.join(pkg_share, 'config', 'behaviors_demo.yaml')
    default_calibration_yaml= os.path.join(pkg_share, 'config', 'blossom_calibration.yaml')

    topic_arg   = DeclareLaunchArgument('topic',   default_value='/blossom_position_controller/commands')
    rate_arg    = DeclareLaunchArgument('rate_hz', default_value='50.0')
    beh_arg     = DeclareLaunchArgument('behavior_yaml',    default_value=default_behavior_yaml)
    calib_arg   = DeclareLaunchArgument('calibration_yaml', default_value=default_calibration_yaml)
    base_arg    = DeclareLaunchArgument('base_neutral_deg', default_value='180.0')

    player = Node(
        package='blossom_dxl_hw',
        executable='blossom_yaml_player.py',
        name='blossom_yaml_player',
        output='screen',
        parameters=[{
            'topic':               LaunchConfiguration('topic'),
            'rate_hz':             LaunchConfiguration('rate_hz'),
            'behavior_yaml':       LaunchConfiguration('behavior_yaml'),
            'calibration_yaml':    LaunchConfiguration('calibration_yaml'),
            'base_neutral_deg':    LaunchConfiguration('base_neutral_deg'),
        }]
    )

    return LaunchDescription([
        topic_arg, rate_arg, beh_arg, calib_arg, base_arg,
        player
    ])
