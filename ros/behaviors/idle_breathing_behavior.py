#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import math
import time


JOINTS = ["head_left", "head_right"]


class BreathingBehavior(Node):
    def __init__(self):
        super().__init__('blossom_idle_behavior')
        self.pub = self.create_publisher(JointTrajectory, '/joint_trajectory_controller/joint_trajectory', 10)
        self.timer = self.create_timer(0.1, self.loop)
        self.start = time.time()


    def loop(self):
        t = time.time() - self.start
        offset = 0.15 * math.sin(0.5 * t)
        msg = JointTrajectory()
        msg.joint_names = JOINTS
        pt = JointTrajectoryPoint()
        pt.positions = [offset, -offset]
        pt.time_from_start.sec = 1
        msg.points.append(pt)
        self.pub.publish(msg)


if __name__ == '__main__':
    rclpy.init()
    node = BreathingBehavior()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()