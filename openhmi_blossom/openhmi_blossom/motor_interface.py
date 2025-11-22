#!/usr/bin/env python3
"""
Motor Interface Node for Blossom Robot
Handles low-level communication with Dynamixel motors via serial
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64MultiArray
import serial
import yaml
import time
from typing import Dict, List, Optional


class MotorInterface(Node):
    """Interface for controlling Blossom's Dynamixel motors."""
    
    def __init__(self):
        super().__init__('motor_interface')
        
        # Declare parameters
        self.declare_parameter('port', '/dev/ttyACM0')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('motor_config', '')
        self.declare_parameter('publish_rate', 50.0)
        
        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        config_file = self.get_parameter('motor_config').value
        publish_rate = self.get_parameter('publish_rate').value
        
        # Initialize serial connection
        try:
            self.serial_port = serial.Serial(port, baudrate, timeout=0.1)
            self.get_logger().info(f'Connected to motors on {port} at {baudrate} baud')
        except serial.SerialException as e:
            self.get_logger().error(f'Failed to open serial port: {e}')
            raise
        
        # Load motor configuration
        self.motor_ids = {}
        self.motor_limits = {}
        if config_file:
            self.load_motor_config(config_file)
        else:
            # Default Blossom configuration
            self.motor_ids = {
                'tower_1': 1,
                'tower_2': 2,
                'tower_3': 3,
                'tower_4': 4,
            }
            self.motor_limits = {
                'tower_1': (0, 4095),
                'tower_2': (0, 4095),
                'tower_3': (0, 4095),
                'tower_4': (0, 4095),
            }
        
        # Current joint positions
        self.current_positions = {name: 2048 for name in self.motor_ids.keys()}
        
        # Publishers
        self.joint_state_pub = self.create_publisher(
            JointState,
            'joint_states',
            10
        )
        
        # Subscribers
        self.joint_cmd_sub = self.create_subscription(
            JointState,
            'joint_commands',
            self.joint_command_callback,
            10
        )
        
        # Timer for publishing joint states
        self.create_timer(1.0 / publish_rate, self.publish_joint_states)
        
        self.get_logger().info('Motor interface initialized')
    
    def load_motor_config(self, config_file: str):
        """Load motor configuration from YAML file."""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                self.motor_ids = config.get('motor_ids', {})
                self.motor_limits = config.get('motor_limits', {})
                self.get_logger().info(f'Loaded motor config from {config_file}')
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')
    
    def joint_command_callback(self, msg: JointState):
        """Handle incoming joint position commands."""
        for i, name in enumerate(msg.name):
            if name in self.motor_ids and i < len(msg.position):
                motor_id = self.motor_ids[name]
                position = msg.position[i]
                
                # Convert position to motor units (0-4095 for Dynamixel)
                motor_position = self.position_to_motor_units(name, position)
                
                # Send command to motor
                self.set_motor_position(motor_id, motor_position)
                self.current_positions[name] = motor_position
    
    def position_to_motor_units(self, joint_name: str, position: float) -> int:
        """
        Convert joint position (radians or normalized) to motor units.
        
        Args:
            joint_name: Name of the joint
            position: Position value (assumed to be in range [-pi, pi] or [0, 1])
        
        Returns:
            Motor position in units (0-4095)
        """
        limits = self.motor_limits.get(joint_name, (0, 4095))
        
        # If position is in radians (assume -pi to pi)
        if abs(position) > 2.0:  # Likely in radians
            # Normalize to 0-1
            normalized = (position + 3.14159) / (2 * 3.14159)
        else:
            # Assume already normalized or close to it
            normalized = (position + 1.0) / 2.0
        
        # Clamp to 0-1
        normalized = max(0.0, min(1.0, normalized))
        
        # Scale to motor range
        motor_pos = int(limits[0] + normalized * (limits[1] - limits[0]))
        return motor_pos
    
    def set_motor_position(self, motor_id: int, position: int):
        """
        Send position command to a motor.
        
        Note: This is a simplified protocol. You'll need to implement
        the actual Dynamixel protocol or use pyserial with proper packet format.
        """
        try:
            # Simplified command format - replace with actual Dynamixel protocol
            # For Dynamixel AX/MX series, you'd use:
            # - Instruction packet with goal position
            cmd = f"#{motor_id}P{position}\r".encode()
            self.serial_port.write(cmd)
            
            # For actual Dynamixel implementation, use dynamixel_sdk:
            # self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, 
            #                                    ADDR_GOAL_POSITION, position)
            
        except Exception as e:
            self.get_logger().error(f'Failed to set motor {motor_id} position: {e}')
    
    def publish_joint_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        
        for name, position in self.current_positions.items():
            msg.name.append(name)
            # Convert motor units back to radians
            radians = self.motor_units_to_radians(position)
            msg.position.append(radians)
            msg.velocity.append(0.0)  # TODO: Read actual velocity
            msg.effort.append(0.0)    # TODO: Read actual effort
        
        self.joint_state_pub.publish(msg)
    
    def motor_units_to_radians(self, motor_position: int) -> float:
        """Convert motor units (0-4095) to radians."""
        normalized = motor_position / 4095.0
        radians = (normalized * 2 * 3.14159) - 3.14159
        return radians
    
    def destroy_node(self):
        """Clean up resources."""
        if hasattr(self, 'serial_port') and self.serial_port.is_open:
            self.serial_port.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = MotorInterface()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
