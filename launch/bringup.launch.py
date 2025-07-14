from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch.substitutions import Command
import os

def generate_launch_description():
    pkg_path = get_package_share_directory('xl330_ros2_control')
    urdf_file = os.path.join(pkg_path, 'description', 'xl330.urdf.xacro')
    config_file = os.path.join(pkg_path, 'config', 'xl330_controllers.yaml')

    return LaunchDescription([
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': Command(['xacro ', urdf_file])}]
        ),

        Node(
            package='controller_manager',
            executable='ros2_control_node',
            parameters=[config_file, {'robot_description': Command(['xacro ', urdf_file])}],
            output='screen'
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_state_broadcaster'],
            output='screen',
        ),

        Node(
            package='controller_manager',
            executable='spawner',
            arguments=['joint_trajectory_controller'],
            output='screen',
        )
    ])