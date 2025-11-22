#!/usr/bin/env python3
"""
Sequence Player Node for Blossom Robot
Plays back pre-recorded gesture sequences
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer
from sensor_msgs.msg import JointState
from std_msgs.msg import String
from control_msgs.action import FollowJointTrajectory
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
import yaml
import json
import os
from typing import Dict, List, Optional
from threading import Lock


class SequencePlayer(Node):
    """Node for playing gesture sequences on Blossom robot."""
    
    def __init__(self):
        super().__init__('sequence_player')
        
        # Declare parameters
        self.declare_parameter('sequences_dir', '')
        self.declare_parameter('default_duration', 1.0)
        self.declare_parameter('interpolation_points', 10)
        
        # Get parameters
        self.sequences_dir = self.get_parameter('sequences_dir').value
        self.default_duration = self.get_parameter('default_duration').value
        self.interpolation_points = self.get_parameter('interpolation_points').value
        
        # Load sequences
        self.sequences = {}
        self.sequence_lock = Lock()
        if self.sequences_dir and os.path.exists(self.sequences_dir):
            self.load_sequences()
        
        # Current playback state
        self.current_sequence = None
        self.is_playing = False
        
        # Publishers
        self.joint_cmd_pub = self.create_publisher(
            JointState,
            'joint_commands',
            10
        )
        
        self.status_pub = self.create_publisher(
            String,
            'sequence_status',
            10
        )
        
        # Subscribers
        self.play_sequence_sub = self.create_subscription(
            String,
            'play_sequence',
            self.play_sequence_callback,
            10
        )
        
        # Services
        self.create_service(
            from std_srvs.srv import Trigger,
            'list_sequences',
            self.list_sequences_callback
        )
        
        self.get_logger().info('Sequence player initialized')
        self.get_logger().info(f'Loaded {len(self.sequences)} sequences')
    
    def load_sequences(self):
        """Load all sequence files from the sequences directory."""
        try:
            for filename in os.listdir(self.sequences_dir):
                if filename.endswith('.yaml') or filename.endswith('.yml'):
                    filepath = os.path.join(self.sequences_dir, filename)
                    seq_name = os.path.splitext(filename)[0]
                    
                    with open(filepath, 'r') as f:
                        sequence_data = yaml.safe_load(f)
                        self.sequences[seq_name] = sequence_data
                        self.get_logger().info(f'Loaded sequence: {seq_name}')
                        
                elif filename.endswith('.json'):
                    filepath = os.path.join(self.sequences_dir, filename)
                    seq_name = os.path.splitext(filename)[0]
                    
                    with open(filepath, 'r') as f:
                        sequence_data = json.load(f)
                        self.sequences[seq_name] = sequence_data
                        self.get_logger().info(f'Loaded sequence: {seq_name}')
                        
        except Exception as e:
            self.get_logger().error(f'Error loading sequences: {e}')
    
    def play_sequence_callback(self, msg: String):
        """Handle request to play a sequence."""
        sequence_name = msg.data
        
        with self.sequence_lock:
            if sequence_name not in self.sequences:
                self.get_logger().warn(f'Unknown sequence: {sequence_name}')
                return
            
            if self.is_playing:
                self.get_logger().warn('Already playing a sequence')
                return
            
            self.is_playing = True
            self.current_sequence = sequence_name
        
        # Play the sequence
        self.play_sequence(sequence_name)
        
        with self.sequence_lock:
            self.is_playing = False
            self.current_sequence = None
    
    def play_sequence(self, sequence_name: str):
        """
        Play a gesture sequence.
        
        Args:
            sequence_name: Name of the sequence to play
        """
        sequence = self.sequences[sequence_name]
        
        self.get_logger().info(f'Playing sequence: {sequence_name}')
        
        # Publish status
        status_msg = String()
        status_msg.data = f'playing:{sequence_name}'
        self.status_pub.publish(status_msg)
        
        # Extract keyframes
        keyframes = sequence.get('keyframes', [])
        if not keyframes:
            self.get_logger().warn(f'No keyframes in sequence: {sequence_name}')
            return
        
        # Play each keyframe
        for i, keyframe in enumerate(keyframes):
            if not self.is_playing:  # Allow interruption
                break
            
            # Extract joint positions
            joint_names = list(keyframe.get('joints', {}).keys())
            joint_positions = list(keyframe.get('joints', {}).values())
            
            # Get duration for this keyframe
            duration = keyframe.get('duration', self.default_duration)
            
            # Interpolate and publish
            self.interpolate_and_publish(joint_names, joint_positions, duration)
            
        # Publish completion status
        status_msg.data = f'completed:{sequence_name}'
        self.status_pub.publish(status_msg)
        self.get_logger().info(f'Completed sequence: {sequence_name}')
    
    def interpolate_and_publish(self, joint_names: List[str], 
                                target_positions: List[float], 
                                duration: float):
        """
        Interpolate between current and target positions and publish commands.
        
        Args:
            joint_names: List of joint names
            target_positions: Target positions for each joint
            duration: Time to complete the motion
        """
        # Simple linear interpolation
        num_steps = self.interpolation_points
        dt = duration / num_steps
        
        for step in range(num_steps + 1):
            if not self.is_playing:
                break
            
            # Create joint state message
            msg = JointState()
            msg.header.stamp = self.get_clock().now().to_msg()
            msg.name = joint_names
            
            # Linear interpolation
            alpha = step / num_steps
            msg.position = [pos * alpha for pos in target_positions]
            
            # Publish
            self.joint_cmd_pub.publish(msg)
            
            # Wait
            if step < num_steps:
                self.get_clock().sleep_for(rclpy.duration.Duration(seconds=dt))
    
    def list_sequences_callback(self, request, response):
        """Service callback to list available sequences."""
        sequence_list = ', '.join(self.sequences.keys())
        response.success = True
        response.message = f'Available sequences: {sequence_list}'
        return response


def main(args=None):
    rclpy.init(args=args)
    node = SequencePlayer()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
