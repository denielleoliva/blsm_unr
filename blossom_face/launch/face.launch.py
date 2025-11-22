#!/usr/bin/env python3
"""
Launch file for Blossom face display
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Generate launch description for face display."""
    
    # Declare launch arguments
    display_type_arg = DeclareLaunchArgument(
        'display_type',
        default_value='web',
        description='Display type: "pygame" or "web"'
    )
    
    port_arg = DeclareLaunchArgument(
        'port',
        default_value='8080',
        description='Port for web server (web display only)'
    )
    
    fullscreen_arg = DeclareLaunchArgument(
        'fullscreen',
        default_value='true',
        description='Run in fullscreen mode (pygame only)'
    )
    
    # Web-based face animator node (recommended for TFT display)
    face_animator_node = Node(
        package='blossom_face',
        executable='face_animator',
        name='face_animator',
        output='screen',
        parameters=[{
            'port': LaunchConfiguration('port'),
        }]
    )
    
    # Pygame-based face display node (alternative)
    # Uncomment to use pygame instead of web
    # face_display_node = Node(
    #     package='blossom_face',
    #     executable='face_display',
    #     name='face_display',
    #     output='screen',
    #     parameters=[{
    #         'fullscreen': LaunchConfiguration('fullscreen'),
    #         'width': 800,
    #         'height': 480,
    #         'fps': 30
    #     }]
    # )
    
    return LaunchDescription([
        display_type_arg,
        port_arg,
        fullscreen_arg,
        face_animator_node,
    ])
