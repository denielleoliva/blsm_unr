from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os
from xacro import process_file

def generate_launch_description():
    pkg_share = get_package_share_directory('blossom_dxl_hw')
    urdf_path = os.path.join(pkg_share, 'urdf', 'blossom.urdf.xacro')
    controllers_yaml = os.path.join(pkg_share, 'config', 'controllers.yaml')

    robot_description = {'robot_description': process_file(urdf_path).toxml()}

    rsp = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[robot_description]
    )

    cm = Node(
        package='controller_manager',
        executable='ros2_control_node',
        parameters=[robot_description, controllers_yaml],
        output='screen'
    )

    sp_js = Node(package='controller_manager', executable='spawner', arguments=['joint_state_broadcaster'], output='screen')
    sp_pos = Node(package='controller_manager', executable='spawner', arguments=['blossom_position_controller'], output='screen')

    return LaunchDescription([rsp, cm, sp_js, sp_pos])
