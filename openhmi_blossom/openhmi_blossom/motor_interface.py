#!/usr/bin/env python3
"""
Motor Interface Node for Blossom Robot
Handles low-level communication with XL-320 Dynamixel motors via serial
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import serial
import yaml
import time
from typing import Dict, List, Optional

# Dynamixel SDK
try:
    from dynamixel_sdk import *
    DYNAMIXEL_SDK_AVAILABLE = True
except ImportError:
    DYNAMIXEL_SDK_AVAILABLE = False
    print("Warning: Dynamixel SDK not available. Motor control will not work.")


class MotorInterface(Node):
    """Interface for controlling Blossom's XL-320 Dynamixel motors."""

    def __init__(self):
        super().__init__('motor_interface')

        # Declare parameters
        self.declare_parameter('port', '/dev/ttyUSB1')
        self.declare_parameter('baudrate', 1000000)
        self.declare_parameter('motor_config', '')
        self.declare_parameter('publish_rate', 50.0)

        # Get parameters
        port = self.get_parameter('port').value
        baudrate = self.get_parameter('baudrate').value
        config_file = self.get_parameter('motor_config').value
        publish_rate = self.get_parameter('publish_rate').value

        # XL-320 specific constants
        self.MOTOR_MODEL = 'XL-320'
        self.POSITION_RANGE = 1023  # XL-320 range is 0-1023, NOT 0-4095!
        self.PROTOCOL_VERSION = 2.0  # XL-320 uses Protocol 2.0

        # XL-320 Control Table addresses
        self.ADDR_TORQUE_ENABLE = 24
        self.ADDR_TORQUE_LIMIT = 35
        self.ADDR_GOAL_POSITION = 30
        self.ADDR_MOVING_SPEED = 32
        self.ADDR_PRESENT_POSITION = 37
        self.ADDR_CW_ANGLE_LIMIT = 6
        self.ADDR_CCW_ANGLE_LIMIT = 8

        # Dynamixel SDK setup
        self.port_handler = None
        self.packet_handler = None

        if DYNAMIXEL_SDK_AVAILABLE:
            # Initialize PortHandler and PacketHandler
            self.port_handler = PortHandler(port)
            self.packet_handler = PacketHandler(self.PROTOCOL_VERSION)

            # Open port
            if self.port_handler.openPort():
                self.get_logger().info(f'Opened port {port}')
            else:
                self.get_logger().error(f'Failed to open port {port}')
                raise RuntimeError(f'Failed to open port {port}')

            # Set baudrate
            if self.port_handler.setBaudRate(baudrate):
                self.get_logger().info(f'Set baudrate to {baudrate}')
            else:
                self.get_logger().error(f'Failed to set baudrate to {baudrate}')
                raise RuntimeError(f'Failed to set baudrate to {baudrate}')

            self.get_logger().info(f'Connected to XL-320 motors on {port} at {baudrate} baud')
        else:
            self.get_logger().warn('Dynamixel SDK not available - running in simulation mode')

        # Load motor configuration
        self.motor_ids = {}
        self.motor_limits = {}
        self.motor_speeds = {}
        if config_file:
            self.load_motor_config(config_file)
        else:
            # Default Blossom configuration for XL-320
            # Motor IDs for Dynamixel servos
            self.motor_ids = {
                'lazy_susan': 1,        # Base rotation motor
                'motor_front': 2,       # Front head plate motor
                'motor_back_left': 3,   # Back-left head plate motor
                'motor_back_right': 4,  # Back-right head plate motor
            }
            # Read position limits from motor hardware (CW/CCW angle limit registers)
            self.motor_limits = {}
            for name, motor_id in self.motor_ids.items():
                if DYNAMIXEL_SDK_AVAILABLE and self.port_handler:
                    dxl_min, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                        self.port_handler, motor_id, self.ADDR_CW_ANGLE_LIMIT
                    )
                    dxl_max, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                        self.port_handler, motor_id, self.ADDR_CCW_ANGLE_LIMIT
                    )
                    if dxl_comm_result == COMM_SUCCESS and dxl_error == 0:
                        self.motor_limits[name] = (dxl_min, dxl_max)
                        self.get_logger().info(f'Read limits for {name} (ID: {motor_id}): {dxl_min} - {dxl_max}')
                    else:
                        self.motor_limits[name] = (0, self.POSITION_RANGE)
                        self.get_logger().warn(f'Could not read limits for {name} (ID: {motor_id}), using default (0, {self.POSITION_RANGE})')
                else:
                    self.motor_limits[name] = (0, self.POSITION_RANGE)


        # Current joint positions (initialize to home/neutral positions)
        self.current_positions = {}
        for name in self.motor_ids.keys():
            if name == 'lazy_susan':
                # Center position for lazy susan (512 for XL-320)
                self.current_positions[name] = 512
            else:
                # Home position for head motors (0 = no pull)
                self.current_positions[name] = 0

        # Initialize motors on startup
        if DYNAMIXEL_SDK_AVAILABLE and self.port_handler:
            self._initialize_motors()

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
        
        # Service for listing joint limits
        self.create_service(
            Trigger,
            'list_joint_limits',
            self.list_joint_limits_callback
        )

        # Timer for publishing joint states
        self.create_timer(1.0 / publish_rate, self.publish_joint_states)

        self.get_logger().info(f'Motor interface initialized for {self.MOTOR_MODEL}')

    def _initialize_motors(self):
        """Initialize all motors on startup."""
        for name, motor_id in self.motor_ids.items():
            try:
                
                # Set moving speed for each motor
                dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, motor_id, self.ADDR_MOVING_SPEED, self.motor_speeds.get(name, 200)  # Default speed if not specified
                )
                
                if (dxl_comm_result != COMM_SUCCESS) or (dxl_error != 0):
                    self.get_logger().error(
                        f'Failed to set moving speed for motor {name} (ID: {motor_id}): '
                        f'{self.packet_handler.getTxRxResult(dxl_comm_result)}, '
                        f'{self.packet_handler.getRxPacketError(dxl_error)}'
                    )
                    continue

                # Read motor position limits from memory locations 6 and 8
                dxl_min_position, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                    self.port_handler, motor_id, 6  # Address for CW Angle Limit (min position)
                )
                dxl_max_position, dxl_comm_result, dxl_error = self.packet_handler.read2ByteTxRx(
                    self.port_handler, motor_id, 8  # Address for CCW Angle Limit (max position)
                )
                
                if (dxl_comm_result != COMM_SUCCESS) or (dxl_error != 0):
                    self.get_logger().error(
                        f'Failed to read position limits for motor {name} (ID: {motor_id}): '
                        f'{self.packet_handler.getTxRxResult(dxl_comm_result)}, '
                        f'{self.packet_handler.getRxPacketError(dxl_error)}'
                    )
                    continue

                # Enable torque
                dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
                   self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 1
                )
                
                if (dxl_comm_result != COMM_SUCCESS) or (dxl_error != 0):
                    self.get_logger().error(
                        f'Failed to enable torque for motor {name} (ID: {motor_id}): '
                        f'{self.packet_handler.getTxRxResult(dxl_comm_result)}, '
                        f'{self.packet_handler.getRxPacketError(dxl_error)}'
                    )
                    continue

                dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                    self.port_handler, motor_id, self.ADDR_TORQUE_LIMIT, 800
                )

                if dxl_comm_result == COMM_SUCCESS:
                    # Store the read position limits as the real position limits
                    self.motor_limits[name] = (dxl_min_position, dxl_max_position)
                    self.get_logger().info(f'Initialized motor {name} (ID: {motor_id})')
                    self.get_logger().info(f'Motor {name} (ID: {motor_id}) position limits: {dxl_min_position} - {dxl_max_position}')
                else:
                    self.get_logger().warn(f'Failed to initialize motor {name} (ID: {motor_id})')
            except Exception as e:
                self.get_logger().error(f'Exception initializing motor {name}: {e}')

    def load_motor_config(self, config_file: str):
        """Load motor configuration from YAML file."""
        try:
            with open(config_file, 'r') as f:
                config = yaml.safe_load(f)
                self.motor_ids = config.get('motor_ids', {})
                self.motor_limits = config.get('motor_limits', {})
                self.motor_speeds = config.get('motor_speeds', {})
                self.get_logger().info(f'Loaded motor config from {config_file}')
                
        except Exception as e:
            self.get_logger().error(f'Failed to load config: {e}')

    def joint_command_callback(self, msg: JointState):
        """Handle incoming joint position commands."""
        for i, name in enumerate(msg.name):
            if name in self.motor_ids and i < len(msg.position):
                motor_id = self.motor_ids[name]
                position = msg.position[i]
                
                # check if mempory 50 Hardware Error Status is in error state, if so, 
                # skip sending command to motor and queue a restart of the motor interface node to clear the buffer
                dxl_hw_error_status, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
                    self.port_handler, motor_id, 50
                )

                if dxl_comm_result != COMM_SUCCESS or dxl_hw_error_status != 0:
                    self.get_logger().error(f'Motor {motor_id} hardware error status: {dxl_hw_error_status}')
                    return
                # Read overload status from address 50 (Present Load)
                dxl_present_load, dxl_comm_result, dxl_error = self.packet_handler.read1ByteTxRx(
                    self.port_handler, motor_id, 50
                )

                if dxl_comm_result != COMM_SUCCESS or dxl_present_load > 800:  # Threshold can be adjusted
                    self.get_logger().warn(f'Motor {motor_id} overloaded, skipping command')
                    return
                
                # Convert position to motor units (0-1023 for XL-320)
                motor_position = self.position_to_motor_units(name, position)

                # Send command to motor
                self.set_motor_position(motor_id, motor_position)
                self.current_positions[name] = motor_position

    def position_to_motor_units(self, joint_name: str, position: float) -> int:
        """
        Convert joint position from controller units to motor units.
        Controller position is already in the motor's native coordinate system.

        Args:
            joint_name: Name of the joint
            position: Position value from controller (in motor's limit range)

        Returns:
            Motor position (clamped to motor's limits)
        """
        limits = self.motor_limits.get(joint_name, (0, 1023))

        # Clamp to motor's defined limits and convert to int
        motor_pos = max(limits[0], min(limits[1], int(position)))

        return motor_pos

    def set_motor_position(self, motor_id: int, position: int):
        """
        Send position command to an XL-320 motor.

        Args:
            motor_id: Dynamixel motor ID
            position: Goal position in XL-320 units (0-1023)
        """
        if not DYNAMIXEL_SDK_AVAILABLE or not self.port_handler:
            # Simulation mode - just log
            self.get_logger().debug(f'[SIM] Motor {motor_id} -> {position}')
            return

        try:
            # Write goal position (2 bytes for XL-320)
            dxl_comm_result, dxl_error = self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, self.ADDR_GOAL_POSITION, position
            )

            if dxl_comm_result != COMM_SUCCESS:
                self.get_logger().error(
                    f'Failed to set motor {motor_id} position: '
                    f'{self.packet_handler.getTxRxResult(dxl_comm_result)}'
                )
            elif dxl_error != 0:
                self.get_logger().error(
                    f'Motor {motor_id} error: '
                    f'{self.packet_handler.getRxPacketError(dxl_error)}'
                )
            else:
                self.get_logger().debug(f'Motor {motor_id} set to position {position}')

        except Exception as e:
            self.get_logger().error(f'Exception setting motor {motor_id} position: {e}')

    def publish_joint_states(self):
        """Publish current joint states."""
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()

        for name, motor_position in self.current_positions.items():
            msg.name.append(name)
            # Convert motor units back to controller units
            controller_pos = self.motor_units_to_controller(name, motor_position)
            msg.position.append(controller_pos)
            msg.velocity.append(0.0)  # TODO: Read actual velocity
            msg.effort.append(0.0)    # TODO: Read actual effort

        self.joint_state_pub.publish(msg)

    def motor_units_to_controller(self, joint_name: str, motor_position: int) -> float:
        """
        Convert motor position back to controller units.
        Motor position is already in the motor's native coordinate system.

        Args:
            joint_name: Name of the joint
            motor_position: Position in motor's native units

        Returns:
            Position in controller units (same as motor's native range)
        """
        # Motor position is already in the correct coordinate system
        return float(motor_position)

    def destroy_node(self):
        """Clean up resources."""
        if DYNAMIXEL_SDK_AVAILABLE and self.port_handler is not None:
            # Disable torque on all motors before closing (XL-320 address)
            for motor_id in self.motor_ids.values():
                try:
                    self.packet_handler.write1ByteTxRx(
                        self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 0
                    )
                except:
                    pass
            # Close port
            self.port_handler.closePort()
            self.get_logger().info('Closed Dynamixel port')
        super().destroy_node()
        
    def list_joint_limits_callback(self, request, response):
        """Service callback to list joint limits."""
        # List limits for all joints
        limits_info = {name: self.motor_limits.get(name, (0, 1023)) for name in self.motor_ids.keys()}
            
        for name, limits in limits_info.items():
            nl = '\n' if response.message else ''
            response.message += f'{nl}{name}: {limits[0]} - {limits[1]}'
        response.success = True
        return response


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
