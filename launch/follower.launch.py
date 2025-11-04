from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('blossom_dxl_hw')
    default_calib = os.path.join(pkg_share, 'config', 'blossom_calibration.yaml')

    return LaunchDescription([
        DeclareLaunchArgument('image_topic', default_value='/camera/image_raw'),
        DeclareLaunchArgument('cmd_topic',   default_value='/blossom_position_controller/commands'),
        DeclareLaunchArgument('calibration', default_value=default_calib),
        DeclareLaunchArgument('debug_image', default_value='false'),

        Node(
            package='blossom_dxl_hw',
            executable='blossom_gaze_follow.py',
            name='blossom_gaze_follow',
            output='screen',
            parameters=[{
                'image_topic': LaunchConfiguration('image_topic'),
                'cmd_topic':   LaunchConfiguration('cmd_topic'),
                'calibration_yaml': LaunchConfiguration('calibration'),
                'base_neutral_deg': 180.0,
                'base_limits_deg': [140.0, 220.0],
                'rate_hz': 30.0,
                'detect_interval': 10,
                'debug_image': LaunchConfiguration('debug_image'),
                'yaw_kp': 35.0, 'yaw_ki': 0.0,
                'pitch_kp': 18.0, 'pitch_ki': 0.0,
                'roll_kp': 6.0,
            }]
        )
    ])
