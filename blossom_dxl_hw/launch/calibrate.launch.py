from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='blossom_dxl_hw',
            executable='calibrate_node',
            name='blossom_calibrate_cpp',
            output='screen',
            parameters=[{
                'port': '/dev/ttyUSB1',
                'baud': 57600,
                'ids': [1, 2, 3, 4],
                'names': [
                    'base_pan',
                    'string_front',
                    'string_back_right',
                    'string_back_left'
                ],
                'outfile': '~/blsm_ws/src/blossom_dxl_hw/config/blossom_calibration.yaml'
            }]
        )
    ])
