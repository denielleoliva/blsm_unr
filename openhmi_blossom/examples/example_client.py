#!/usr/bin/env python3
"""
Example client for controlling Blossom robot
Demonstrates various ways to interact with the robot
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import Pose
from sensor_msgs.msg import JointState
import time
import math


class BlossomExampleClient(Node):
    """Example client showing different control methods."""
    
    def __init__(self):
        super().__init__('blossom_example_client')
        
        # Publishers
        self.sequence_pub = self.create_publisher(String, 'play_sequence', 10)
        self.behavior_pub = self.create_publisher(String, 'behavior_command', 10)
        self.pose_pub = self.create_publisher(Pose, 'target_pose', 10)
        self.joint_pub = self.create_publisher(JointState, 'joint_commands', 10)
        
        # Wait for publishers to be ready
        time.sleep(1.0)
        
        self.get_logger().info('Blossom example client ready')
    
    def play_sequence(self, sequence_name: str):
        """Play a pre-defined gesture sequence."""
        self.get_logger().info(f'Playing sequence: {sequence_name}')
        msg = String()
        msg.data = sequence_name
        self.sequence_pub.publish(msg)
    
    def execute_behavior(self, behavior: str):
        """Execute a high-level behavior."""
        self.get_logger().info(f'Executing behavior: {behavior}')
        msg = String()
        msg.data = behavior
        self.behavior_pub.publish(msg)
    
    def set_pose(self, pitch: float, yaw: float, roll: float, height: float):
        """
        Set robot pose directly.
        
        Args:
            pitch: Pitch angle in radians
            yaw: Yaw angle in radians
            roll: Roll angle in radians
            height: Height (0.0 to 1.0)
        """
        self.get_logger().info(
            f'Setting pose: pitch={pitch:.2f}, yaw={yaw:.2f}, '
            f'roll={roll:.2f}, height={height:.2f}'
        )
        
        # Convert euler angles to quaternion
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)
        
        msg = Pose()
        msg.position.z = height
        msg.orientation.w = cr * cp * cy + sr * sp * sy
        msg.orientation.x = sr * cp * cy - cr * sp * sy
        msg.orientation.y = cr * sp * cy + sr * cp * sy
        msg.orientation.z = cr * cp * sy - sr * sp * cy
        
        self.pose_pub.publish(msg)
    
    def set_joint_positions(self, positions: dict):
        """
        Set joint positions directly.
        
        Args:
            positions: Dictionary mapping joint names to positions
        """
        self.get_logger().info(f'Setting joint positions: {positions}')
        
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = list(positions.keys())
        msg.position = list(positions.values())
        
        self.joint_pub.publish(msg)
    
    def demo_gestures(self):
        """Run through a demo of different gestures."""
        self.get_logger().info('Starting gesture demo...')
        
        gestures = ['yes', 'no', 'happy', 'look_left', 'look_right']
        
        for gesture in gestures:
            self.play_sequence(gesture)
            time.sleep(3.0)
        
        self.get_logger().info('Gesture demo complete')
    
    def demo_behaviors(self):
        """Run through a demo of different behaviors."""
        self.get_logger().info('Starting behavior demo...')
        
        behaviors = ['nod', 'shake', 'happy', 'excited']
        
        for behavior in behaviors:
            self.execute_behavior(behavior)
            time.sleep(3.0)
        
        self.get_logger().info('Behavior demo complete')
    
    def demo_poses(self):
        """Run through a demo of different poses."""
        self.get_logger().info('Starting pose demo...')

        # Neutral
        self.set_pose(0.0, 0.0, 0.0, 0.5)
        time.sleep(2.0)

        # Look up
        self.set_pose(0.3, 0.0, 0.0, 0.5)
        time.sleep(2.0)

        # Look down
        self.set_pose(-0.3, 0.0, 0.0, 0.5)
        time.sleep(2.0)

        # Look left
        self.set_pose(0.0, 0.5, 0.0, 0.5)
        time.sleep(2.0)

        # Look right
        self.set_pose(0.0, -0.5, 0.0, 0.5)
        time.sleep(2.0)

        # Tilt
        self.set_pose(0.0, 0.0, 0.3, 0.5)
        time.sleep(2.0)

        # Back to neutral
        self.set_pose(0.0, 0.0, 0.0, 0.5)

        self.get_logger().info('Pose demo complete')

    def demo_joints(self):
        """Run through a demo of direct joint control."""
        self.get_logger().info('Starting joint control demo...')
        self.get_logger().info('Motor ranges:')
        self.get_logger().info('  lazy_susan: [0, 1023], home=512')
        self.get_logger().info('  motor_front/back_left/back_right: [0, 400], 0=extended, 400=max pull')

        # Neutral/home position
        self.get_logger().info('Moving to home position...')
        self.set_joint_positions({
            'lazy_susan': 512,      # Center
            'motor_front': 0,       # Extended (no pull)
            'motor_back_left': 0,   # Extended
            'motor_back_right': 0   # Extended
        })
        time.sleep(2.0)

        # Rotate base left
        self.get_logger().info('Rotating base left...')
        self.set_joint_positions({
            'lazy_susan': 400,      # Turn left
            'motor_front': 100,     # Neutral head
            'motor_back_left': 100,
            'motor_back_right': 100
        })
        time.sleep(2.0)

        # Rotate base right
        self.get_logger().info('Rotating base right...')
        self.set_joint_positions({
            'lazy_susan': 624,      # Turn right
            'motor_front': 100,     # Neutral head
            'motor_back_left': 100,
            'motor_back_right': 100
        })
        time.sleep(2.0)

        # Center base, nod down
        self.get_logger().info('Nodding head down...')
        self.set_joint_positions({
            'lazy_susan': 512,      # Center
            'motor_front': 300,     # Pull front down
            'motor_back_left': 50,  # Less pull at back
            'motor_back_right': 50
        })
        time.sleep(2.0)

        # Tilt head up
        self.get_logger().info('Tilting head up...')
        self.set_joint_positions({
            'lazy_susan': 512,      # Center
            'motor_front': 20,      # Minimal pull at front
            'motor_back_left': 250, # More pull at back
            'motor_back_right': 250
        })
        time.sleep(2.0)

        # Combined: turn left and nod
        self.get_logger().info('Turn left and nod...')
        self.set_joint_positions({
            'lazy_susan': 400,      # Turn left
            'motor_front': 250,     # Nod down
            'motor_back_left': 80,
            'motor_back_right': 80
        })
        time.sleep(2.0)

        # Return to home
        self.get_logger().info('Returning to home position...')
        self.set_joint_positions({
            'lazy_susan': 512,
            'motor_front': 0,
            'motor_back_left': 0,
            'motor_back_right': 0
        })
        time.sleep(2.0)

        self.get_logger().info('Joint control demo complete')
    
    def interactive_mode(self):
        """Interactive control mode."""
        self.get_logger().info('Entering interactive mode...')
        self.get_logger().info('Commands:')
        self.get_logger().info('  s <sequence> - Play sequence')
        self.get_logger().info('  b <behavior> - Execute behavior')
        self.get_logger().info('  p <pitch> <yaw> <roll> <height> - Set pose')
        self.get_logger().info('  q - Quit')
        
        while rclpy.ok():
            try:
                cmd = input('> ').strip().split()
                if not cmd:
                    continue
                
                if cmd[0] == 'q':
                    break
                elif cmd[0] == 's' and len(cmd) > 1:
                    self.play_sequence(cmd[1])
                elif cmd[0] == 'b' and len(cmd) > 1:
                    self.execute_behavior(cmd[1])
                elif cmd[0] == 'p' and len(cmd) == 5:
                    pitch = float(cmd[1])
                    yaw = float(cmd[2])
                    roll = float(cmd[3])
                    height = float(cmd[4])
                    self.set_pose(pitch, yaw, roll, height)
                else:
                    self.get_logger().warn('Invalid command')
                    
            except KeyboardInterrupt:
                break
            except Exception as e:
                self.get_logger().error(f'Error: {e}')
        
        self.get_logger().info('Exiting interactive mode')


def main(args=None):
    rclpy.init(args=args)
    client = BlossomExampleClient()

    # Run demos
    try:
        # Uncomment the demo you want to run:

        # Demo 1: Gestures (pre-recorded sequences)
        # client.demo_gestures()

        # Demo 2: Behaviors (high-level commands)
        # client.demo_behaviors()

        # Demo 3: Poses (orientation control)
        # client.demo_poses()

        # Demo 4: Direct joint control (low-level motor commands)
        # client.demo_joints()

        # Interactive mode
        client.interactive_mode()

    except KeyboardInterrupt:
        pass
    finally:
        client.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
