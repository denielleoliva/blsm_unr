#!/usr/bin/env python3
"""
Blossom Controller Node
Main controller for coordinating Blossom robot behaviors
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import Pose
import math
from typing import Dict, Tuple


class BlossomController(Node):
    """High-level controller for Blossom robot."""
    
    def __init__(self):
        super().__init__('blossom_controller')
        
        # Declare parameters
        self.declare_parameter('idle_sequence', 'idle')
        self.declare_parameter('enable_idle', True)
        self.declare_parameter('idle_interval', 10.0)
        
        # Get parameters
        self.idle_sequence = self.get_parameter('idle_sequence').value
        self.enable_idle = self.get_parameter('enable_idle').value
        self.idle_interval = self.get_parameter('idle_interval').value
        
        # Robot state
        self.current_pose = {'pitch': 0.0, 'yaw': 0.0, 'roll': 0.0, 'height': 0.5}
        self.last_command_time = self.get_clock().now()
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )
        
        self.sequence_cmd_pub = self.create_publisher(
            String,
            'play_sequence',
            10
        )
        
        # Subscribers
        self.pose_sub = self.create_subscription(
            Pose,
            'target_pose',
            self.pose_callback,
            10
        )
        
        self.behavior_sub = self.create_subscription(
            String,
            'behavior_command',
            self.behavior_callback,
            10
        )
        
        # Idle behavior timer
        if self.enable_idle:
            self.idle_timer = self.create_timer(
                self.idle_interval,
                self.idle_callback
            )
        
        self.get_logger().info('Blossom controller initialized')
    
    def pose_callback(self, msg: Pose):
        """
        Handle target pose commands.
        Converts quaternion orientation to Blossom's pitch/yaw/roll.
        """
        # Extract position (height)
        height = msg.position.z
        
        # Convert quaternion to euler angles
        # Simple conversion - you may want to use transforms3d or similar
        pitch, yaw, roll = self.quaternion_to_euler(
            msg.orientation.x,
            msg.orientation.y,
            msg.orientation.z,
            msg.orientation.w
        )
        
        self.current_pose = {
            'pitch': pitch,
            'yaw': yaw,
            'roll': roll,
            'height': height
        }
        
        # Convert to joint commands
        self.publish_joint_commands()
        
        # Update last command time
        self.last_command_time = self.get_clock().now()
    
    def quaternion_to_euler(self, x: float, y: float, z: float, w: float) -> Tuple[float, float, float]:
        """Convert quaternion to euler angles (roll, pitch, yaw)."""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return pitch, yaw, roll
    
    def publish_joint_commands(self):
        """
        Convert current pose to joint commands for Blossom's four-motor system.

        Blossom uses 4 motors:
        - lazy_susan: rotates the entire base (controls yaw/turning)
          Range: [0, 1023], home position: 512
        - motor_front: pulls down the front of the head plate
        - motor_back_left: pulls down the back-left of the head plate
        - motor_back_right: pulls down the back-right of the head plate
          Range: [0, 400], 0 = no pull (extended), 400 = max pull (retracted)
        """
        pitch = self.current_pose['pitch']  # Positive = nod down (front pulls more)
        yaw = self.current_pose['yaw']      # Positive = turn right
        height = self.current_pose['height'] # 0.0 to 1.0 (normalized)

        # Lazy susan motor: controls base rotation (yaw)
        # Map yaw angle to motor range [0, 1023] with center at 512
        # Assuming yaw is in radians, scale appropriately
        yaw_scale = 512.0 / math.pi  # Full rotation range
        lazy_susan = 512.0 + (yaw * yaw_scale)
        lazy_susan = max(0.0, min(1023.0, lazy_susan))

        # Head plate motors: control pitch (nodding) only
        # Base position from height (inverted: higher height = less pull)
        # Map height 0.0-1.0 to pull range 400-0
        base_pull = 400 * (1.0 - height)

        # Scale factor for pitch effects
        pitch_scale = 200.0  # How much pitch affects motor difference

        # Calculate motor positions based on pitch only
        # Nod down (positive pitch): front motor pulls more (higher value)
        # Nod up (negative pitch): back motors pull more (higher values)
        motor_front = base_pull + (pitch * pitch_scale)
        motor_back_left = base_pull - (pitch * pitch_scale / 2)
        motor_back_right = base_pull - (pitch * pitch_scale / 2)

        # Clamp head plate motor values to limits [0, 400]
        motor_front = max(0.0, min(400.0, motor_front))
        motor_back_left = max(0.0, min(400.0, motor_back_left))
        motor_back_right = max(0.0, min(400.0, motor_back_right))

        # Create joint state message
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['lazy_susan', 'motor_front', 'motor_back_left', 'motor_back_right']
        msg.position = [lazy_susan, motor_front, motor_back_left, motor_back_right]

        self.joint_cmd_pub.publish(msg)
    
    def behavior_callback(self, msg: String):
        """
        Handle high-level behavior commands.
        
        Examples: 'nod', 'shake_head', 'look_left', etc.
        """
        behavior = msg.data.lower()
        
        self.get_logger().info(f'Executing behavior: {behavior}')
        
        # Map behaviors to sequences
        behavior_map = {
            'nod': 'yes',
            'yes': 'yes',
            'shake': 'no',
            'no': 'no',
            'happy': 'happy',
            'sad': 'sad',
            'excited': 'excited',
            'calm': 'calm',
            'look_left': 'look_left',
            'look_right': 'look_right',
            'look_up': 'look_up',
            'look_down': 'look_down',
        }
        
        sequence_name = behavior_map.get(behavior, behavior)
        
        # Send sequence command
        seq_msg = String()
        seq_msg.data = sequence_name
        self.sequence_cmd_pub.publish(seq_msg)
        
        # Update last command time
        self.last_command_time = self.get_clock().now()
    
    def idle_callback(self):
        """Trigger idle behavior if no recent commands."""
        time_since_last_cmd = (self.get_clock().now() - self.last_command_time).nanoseconds / 1e9
        
        if time_since_last_cmd > self.idle_interval:
            self.get_logger().debug('Triggering idle behavior')
            
            seq_msg = String()
            seq_msg.data = self.idle_sequence
            self.sequence_cmd_pub.publish(seq_msg)


def main(args=None):
    rclpy.init(args=args)
    node = BlossomController()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
