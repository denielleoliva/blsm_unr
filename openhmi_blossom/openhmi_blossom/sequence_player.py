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
from std_srvs.srv import Trigger


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
        self.last_positions = {}  # Track last known positions for interpolation
        
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
            Trigger,
            'list_sequences',
            self.list_sequences_callback
        )
        
        # Service for registering YAML sequences
        # Uses String message type where data contains YAML
        self.register_sequence_sub = self.create_subscription(
            String,
            'register_sequence',
            self.register_sequence_callback,
            10
        )
        
        self.get_logger().info('Sequence player initialized')
        self.get_logger().info(f'Loaded {len(self.sequences)} sequences')
    
    # XL-320 full range: 0–300° = 0–1023 raw = 0–5.236 rad
    _RAD_TO_RAW = 1023.0 / (300.0 * 3.141592653589793 / 180.0)  # ≈ 195.3
    _DOF_MAP = {
        'tower_1': 'motor_front',
        'tower_2': 'motor_back_left',
        'tower_3': 'motor_back_right',
        'base':    'lazy_susan',
    }

    def _convert_blossom_format(self, anim: dict) -> dict:
        """Convert original Blossom JSON animation to internal keyframe format."""
        frames = anim.get('frame_list', [])
        keyframes = []
        prev_millis = None

        for frame in frames:
            millis = frame['millis']
            duration = 0.0 if prev_millis is None else max(0.05, (millis - prev_millis) / 1000.0)

            joints = {}
            for entry in frame.get('positions', []):
                motor = self._DOF_MAP.get(entry['dof'])
                if motor:
                    joints[motor] = max(0, min(1023, round(entry['pos'] * self._RAD_TO_RAW)))

            if joints:
                keyframes.append({'joints': joints, 'duration': round(duration, 4)})

            prev_millis = millis

        return {'keyframes': keyframes}

    def load_sequences(self):
        """Load all sequence files from the sequences directory."""
        try:
            for filename in os.listdir(self.sequences_dir):
                if filename.endswith('.yaml') or filename.endswith('.yml'):
                    filepath = os.path.join(self.sequences_dir, filename)

                    # Skip motor_config.yaml - it's not a sequence file
                    if 'motor_config' in filename:
                        continue

                    with open(filepath, 'r') as f:
                        file_data = yaml.safe_load(f)

                        # Check if this file contains multiple sequences or a single one
                        if isinstance(file_data, dict):
                            # If the file has a 'keyframes' key at top level, it's a single sequence
                            if 'keyframes' in file_data:
                                seq_name = os.path.splitext(filename)[0]
                                self.sequences[seq_name] = file_data
                                self.get_logger().info(f'Loaded sequence: {seq_name}')
                            else:
                                # Otherwise, it contains multiple sequences as top-level keys
                                for seq_name, sequence_data in file_data.items():
                                    # Only load if it's a dict with keyframes
                                    if isinstance(sequence_data, dict) and 'keyframes' in sequence_data:
                                        # Convert sequence name to string to avoid boolean issues
                                        seq_name_str = str(seq_name)
                                        self.sequences[seq_name_str] = sequence_data
                                        self.get_logger().info(f'Loaded sequence: {seq_name_str}')

                elif filename.endswith('.json'):
                    filepath = os.path.join(self.sequences_dir, filename)

                    with open(filepath, 'r') as f:
                        file_data = json.load(f)

                        if isinstance(file_data, dict):
                            # Original Blossom format: {"animation": "name", "frame_list": [...]}
                            if 'frame_list' in file_data:
                                seq_name = file_data.get('animation') or os.path.splitext(filename)[0]
                                self.sequences[seq_name] = self._convert_blossom_format(file_data)
                                self.get_logger().info(f'Loaded Blossom sequence: {seq_name}')
                            elif 'keyframes' in file_data:
                                seq_name = os.path.splitext(filename)[0]
                                self.sequences[seq_name] = file_data
                                self.get_logger().info(f'Loaded sequence: {seq_name}')
                            else:
                                for seq_name, sequence_data in file_data.items():
                                    if isinstance(sequence_data, dict) and 'keyframes' in sequence_data:
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
            joints_dict = keyframe.get('joints', {})
            joint_names = list(joints_dict.keys())
            target_positions = list(joints_dict.values())

            # Get start positions (from last keyframe or use target as fallback)
            start_positions = [
                self.last_positions.get(name, target)
                for name, target in zip(joint_names, target_positions)
            ]

            # Get duration for this keyframe
            duration = keyframe.get('duration', self.default_duration)

            # Interpolate and publish with smooth easing
            self.interpolate_and_publish(joint_names, start_positions, target_positions, duration)

            # Update last known positions
            for name, pos in zip(joint_names, target_positions):
                self.last_positions[name] = pos
            
        # Publish completion status
        status_msg.data = f'completed:{sequence_name}'
        self.status_pub.publish(status_msg)
        self.get_logger().info(f'Completed sequence: {sequence_name}')
    
    def _cubic_ease_in_out(self, t: float) -> float:
        """
        Compute cubic ease-in-out interpolation factor.

        Provides smooth acceleration at the start and deceleration at the end,
        resulting in more natural-looking motion.

        Args:
            t: Normalized time (0.0 to 1.0)

        Returns:
            Eased interpolation factor (0.0 to 1.0)
        """
        if t < 0.5:
            return 4.0 * t * t * t
        else:
            return 1.0 - pow(-2.0 * t + 2.0, 3) / 2.0

    def interpolate_and_publish(self, joint_names: List[str],
                                start_positions: List[float],
                                target_positions: List[float],
                                duration: float):
        """
        Command motors to target positions at a speed calculated from distance/duration.

        Sets the XL-320 MOVING_SPEED register per-joint via the velocity field so the
        motor moves at the right rate and arrives at the target in approximately `duration`
        seconds.  The motor's built-in position controller handles smooth motion; no
        software stepping is needed.

        XL-320 speed conversion:
            speed_units ≈ distance_units * 0.4387 / duration
            (derived from: max speed = 114 RPM = ~2332 position-units/sec at register 1023)

        Args:
            joint_names: List of joint names
            start_positions: Starting positions for each joint
            target_positions: Target positions for each joint
            duration: Time in seconds to complete the motion
        """
        if not self.is_playing:
            return

        # XL-320: 1 speed unit ≈ 114/1023 RPM; 1023 units = 300 deg → conversion factor
        XL320_UNITS_PER_SEC_AT_1023 = 2332.0  # position-units/sec at MOVING_SPEED=1023

        velocities = []
        for start, target in zip(start_positions, target_positions):
            distance = abs(target - start)
            if distance < 1 or duration <= 0:
                # No meaningful movement — keep current motor speed
                velocities.append(0.0)
            else:
                required_units_per_sec = distance / duration
                speed = required_units_per_sec / XL320_UNITS_PER_SEC_AT_1023 * 1023
                # Clamp: 20 minimum avoids stalling; 1023 is hardware max
                velocities.append(float(max(20, min(1023, int(speed)))))

        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = joint_names
        msg.position = [float(p) for p in target_positions]
        msg.velocity = velocities
        self.joint_cmd_pub.publish(msg)

        # Wait for the motor to complete its movement
        self.get_clock().sleep_for(rclpy.duration.Duration(seconds=duration))
    
    def list_sequences_callback(self, request, response):
        """Service callback to list available sequences."""
        sequence_list = ', '.join(self.sequences.keys())
        response.success = True
        response.message = f'Available sequences: {sequence_list}'
        return response
    
    def register_sequence_callback(self, msg: String):
        """Callback to register a sequence from YAML or JSON string."""
        try:
            # Try YAML first, then JSON
            try:
                sequence_data = yaml.safe_load(msg.data)
            except:
                sequence_data = json.loads(msg.data)

            if not isinstance(sequence_data, dict):
                self.get_logger().error('Sequence data must be a dictionary')
                return

            # If it contains a single sequence with 'keyframes', generate a name
            if 'keyframes' in sequence_data:
                sequence_name = f"registered_{len(self.sequences) + 1}"
                self.sequences[sequence_name] = sequence_data
                self.get_logger().info(f"Registered sequence: {sequence_name}")
            else:
                # Otherwise, iterate through all keys and load sequences
                for seq_name, seq_data in sequence_data.items():
                    if isinstance(seq_data, dict) and 'keyframes' in seq_data:
                        self.sequences[seq_name] = seq_data
                        self.get_logger().info(f"Registered sequence: {seq_name}")
        except Exception as e:
            self.get_logger().error(f"Failed to register sequence: {str(e)}")

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
