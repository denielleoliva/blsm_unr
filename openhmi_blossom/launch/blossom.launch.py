#!/usr/bin/env python3
"""
Launch file for Blossom robot system
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
import os


def generate_launch_description():
    """Generate launch description for Blossom robot."""
    
    # Declare launch arguments
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='/dev/ttyACM0',
        description='Serial port for motor communication'
    )
    
    baudrate_arg = DeclareLaunchArgument(
        'baudrate',
        default_value='1000000',
        description='Baudrate for serial communication'
    )
    
    motor_config_arg = DeclareLaunchArgument(
        'motor_config',
        default_value=PathJoinSubstitution([
            FindPackageShare('openhmi_blossom'),
            'config',
            'motor_config.yaml'
        ]),
        description='Path to motor configuration file'
    )
    
    sequences_dir_arg = DeclareLaunchArgument(
        'sequences_dir',
        default_value=PathJoinSubstitution([
            FindPackageShare('openhmi_blossom'),
            'config'
        ]),
        description='Directory containing gesture sequences'
    )
    
    enable_idle_arg = DeclareLaunchArgument(
        'enable_idle',
        default_value='true',
        description='Enable idle behavior'
    )
    
    # Motor interface node
    motor_interface_node = Node(
        package='openhmi_blossom',
        executable='motor_interface',
        name='motor_interface',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
            'baudrate': LaunchConfiguration('baudrate'),
            'motor_config': LaunchConfiguration('motor_config'),
            'publish_rate': 50.0
        }]
    )
    
    # Sequence player node
    sequence_player_node = Node(
        package='openhmi_blossom',
        executable='sequence_player',
        name='sequence_player',
        output='screen',
        parameters=[{
            'sequences_dir': LaunchConfiguration('sequences_dir'),
            'default_duration': 1.0,
            'interpolation_points': 20
        }]
    )
    
    # Blossom controller node
    blossom_controller_node = Node(
        package='openhmi_blossom',
        executable='blossom_controller',
        name='blossom_controller',
        output='screen',
        parameters=[{
            'idle_sequence': 'idle',
            'enable_idle': LaunchConfiguration('enable_idle'),
            'idle_interval': 10.0
        }]
    )
    
    # Gesture action server node
    gesture_server_node = Node(
        package='openhmi_blossom',
        executable='gesture_server',
        name='gesture_server',
        output='screen'
    )
    
    return LaunchDescription([
        port_arg,
        baudrate_arg,
        motor_config_arg,
        sequences_dir_arg,
        enable_idle_arg,
        motor_interface_node,
        sequence_player_node,
        blossom_controller_node,
        gesture_server_node,
    ])
