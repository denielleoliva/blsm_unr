#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import sys
import termios
import tty

JOINTS = ["base_swivel", "string_left", "string_right", "string_back"]

KEY_TO_INDEX = {
    '1': 0,
    '2': 1,
    '3': 2,
    '4': 3
}

class XL330Teleop(Node):
    def __init__(self):
        super().__init__('xl330_teleop')
        self.publisher = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.positions = [0.0] * len(JOINTS)
        self.step = 0.1
        self.get_logger().info("Use keys 1–4 to select joint, 'a'/'d' to move, 'q' to quit.")
        self.current_joint = 0

    def send_command(self):
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        point = JointTrajectoryPoint()
        point.positions = self.positions
        point.time_from_start.sec = 1
        msg.points.append(point)
        self.publisher.publish(msg)
        self.get_logger().info(f"Command: {self.positions}")


def get_key():
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
    try:
        tty.setraw(fd)
        return sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)


def main(args=None):
    rclpy.init(args=args)
    node = XL330Teleop()
    try:
        while rclpy.ok():
            key = get_key()
            if key in KEY_TO_INDEX:
                node.current_joint = KEY_TO_INDEX[key]
                node.get_logger().info(f"Selected joint: {JOINTS[node.current_joint]}")
            elif key == 'a':
                node.positions[node.current_joint] -= node.step
                node.send_command()
            elif key == 'd':
                node.positions[node.current_joint] += node.step
                node.send_command()
            elif key == 'q':
                break
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()