#!/usr/bin/env python3
"""
Motor Interface Node for Blossom Robot
Handles low-level communication with XL-320 Dynamixel motors via serial
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_srvs.srv import Trigger
import yaml
import time
from typing import Optional

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
        self.declare_parameter('port', '/dev/ttyUSB0')
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
        self.ADDR_SHUTDOWN = 18          # EEPROM, 1 byte — bitmask of faults that cut torque
        self.SHUTDOWN_VOLTAGE_BIT = 0x01 # bit 0 = input-voltage error shutdown
        self.ADDR_PRESENT_VOLTAGE = 45   # 1 byte, unit = 0.1 V
        self.ADDR_HW_ERROR_STATUS = 50   # 1 byte, bit 0 = input-voltage error
        self.HW_ERROR_VOLTAGE_BIT = 0x01
        self.VOLTAGE_WARN_THRESHOLD = 65      # 6.5 V — warn before the motor trips (~6.0 V)
        self.VOLTAGE_CRITICAL_THRESHOLD = 62  # 6.2 V — reduce torque to shed current draw
        self.TORQUE_LIMIT_NORMAL = 400        # ~39 % — lower than default 512 to reduce steady-state current
        self.TORQUE_LIMIT_REDUCED = 250       # ~24 % — applied automatically when voltage sags
        self._current_torque_limit = self.TORQUE_LIMIT_NORMAL

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
            # IDs match original pypot/Blossom convention:
            #   1-3 = tower (head-tilt) motors, 4 = base rotation
            self.motor_ids = {
                'lazy_susan':       1,  # Base rotation
                'motor_front':      2,  # Front head-tilt string
                'motor_back_left':  3,  # Back-left head-tilt string
                'motor_back_right': 4,  # Back-right head-tilt string
                'ear':              5,  # Ear motor (CW limit 500, CCW limit 1023)
            }
            # Fallback limits used when hardware register reads fail
            default_limits = {
                'ear': (500, 1023),  # CW stop at 500, CCW stop at 1023
            }

            # Read position limits from motor hardware (CW/CCW angle limit registers)
            self.motor_limits = {}
            for name, motor_id in self.motor_ids.items():
                fallback = default_limits.get(name, (0, self.POSITION_RANGE))
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
                        self.motor_limits[name] = fallback
                        self.get_logger().warn(f'Could not read limits for {name} (ID: {motor_id}), using fallback {fallback}')
                else:
                    self.motor_limits[name] = fallback


        # Current joint positions (initialize to home/neutral positions)
        home = {'lazy_susan': 512, 'motor_front': 870,
                'motor_back_left': 870, 'motor_back_right': 870,
                'ear': 761}
        self.current_positions = {
            name: home.get(name, 0) for name in self.motor_ids.keys()
        }

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

        # Voltage monitoring — check all motors every 5 s and throttle torque if low
        if DYNAMIXEL_SDK_AVAILABLE and self.port_handler:
            self.create_timer(5.0, self._check_voltages)

        self.get_logger().info(f'Motor interface initialized for {self.MOTOR_MODEL}')

    def _apply_torque_limit(self, limit: int):
        """Write a torque limit to all motors."""
        for motor_id in self.motor_ids.values():
            self.packet_handler.write2ByteTxRx(
                self.port_handler, motor_id, self.ADDR_TORQUE_LIMIT, limit
            )

    def _check_voltages(self):
        """Check all motor voltages and reduce torque limit if supply is sagging."""
        critical = False
        warn = False
        for name, motor_id in self.motor_ids.items():
            volts = self._read_voltage(motor_id)
            if volts is None:
                continue
            if volts < self.VOLTAGE_CRITICAL_THRESHOLD * 0.1:
                self.get_logger().warn(f'Motor {name} critical voltage {volts:.1f} V')
                critical = True
            elif volts < self.VOLTAGE_WARN_THRESHOLD * 0.1:
                self.get_logger().warn(f'Motor {name} low voltage {volts:.1f} V')
                warn = True

        if critical:
            if self._current_torque_limit != self.TORQUE_LIMIT_REDUCED:
                self._current_torque_limit = self.TORQUE_LIMIT_REDUCED
                self._apply_torque_limit(self.TORQUE_LIMIT_REDUCED)
                self.get_logger().warn(
                    f'Voltage critical — torque limit reduced to {self.TORQUE_LIMIT_REDUCED}'
                )
        elif not warn and self._current_torque_limit != self.TORQUE_LIMIT_NORMAL:
            self._current_torque_limit = self.TORQUE_LIMIT_NORMAL
            self._apply_torque_limit(self.TORQUE_LIMIT_NORMAL)
            self.get_logger().info(
                f'Voltage recovered — torque limit restored to {self.TORQUE_LIMIT_NORMAL}'
            )

    def _read_voltage(self, motor_id: int) -> Optional[float]:
        """Return present voltage in volts, or None on comm failure."""
        if not DYNAMIXEL_SDK_AVAILABLE or not self.port_handler:
            return None
        raw, result, _ = self.packet_handler.read1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_PRESENT_VOLTAGE
        )
        if result != COMM_SUCCESS:
            return None
        return raw * 0.1

    def _wait_for_motor_ready(self, motor_id: int, timeout: float = 2.0) -> bool:
        """Ping a motor after reboot until it responds or timeout is reached."""
        deadline = time.time() + timeout
        while time.time() < deadline:
            _, result, _ = self.packet_handler.ping(self.port_handler, motor_id)
            if result == COMM_SUCCESS:
                return True
            time.sleep(0.05)
        return False

    def _recover_motor(self, motor_id: int, name: str) -> bool:
        """Reboot a motor to clear a hardware error, then re-initialize it."""
        self.get_logger().warn(f'Attempting recovery reboot for motor {name} (ID: {motor_id})')
        result = self.packet_handler.reboot(self.port_handler, motor_id)
        if result != COMM_SUCCESS:
            self.get_logger().error(f'Reboot failed for motor {name} (ID: {motor_id})')
            return False

        if not self._wait_for_motor_ready(motor_id):
            self.get_logger().error(f'Motor {name} (ID: {motor_id}) did not respond after reboot')
            return False

        # Re-enable torque and restore speed
        self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, self.ADDR_MOVING_SPEED,
            self.motor_speeds.get(name, 200)
        )
        comm, err = self.packet_handler.write1ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_ENABLE, 1
        )
        if comm != COMM_SUCCESS or err != 0:
            self.get_logger().error(f'Failed to re-enable torque after recovery for {name}')
            return False

        self.packet_handler.write2ByteTxRx(
            self.port_handler, motor_id, self.ADDR_TORQUE_LIMIT, self._current_torque_limit
        )
        self.get_logger().info(f'Motor {name} (ID: {motor_id}) recovered successfully')
        return True

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

                # Disable voltage-error shutdown so the motor keeps running under low voltage.
                # Read the current shutdown bitmask and clear only the voltage bit — overheating
                # and encoder-error shutdowns are intentionally kept.
                shutdown_val, comm, _ = self.packet_handler.read1ByteTxRx(
                    self.port_handler, motor_id, self.ADDR_SHUTDOWN
                )
                if comm == COMM_SUCCESS:
                    new_shutdown = shutdown_val & ~self.SHUTDOWN_VOLTAGE_BIT
                    if new_shutdown != shutdown_val:
                        self.packet_handler.write1ByteTxRx(
                            self.port_handler, motor_id, self.ADDR_SHUTDOWN, new_shutdown
                        )
                        self.get_logger().info(
                            f'Motor {name} (ID: {motor_id}) voltage shutdown disabled '
                            f'(shutdown reg: 0x{shutdown_val:02x} -> 0x{new_shutdown:02x})'
                        )

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
                    self.port_handler, motor_id, self.ADDR_TORQUE_LIMIT, self._current_torque_limit
                )

                if dxl_comm_result == COMM_SUCCESS:
                    # Store the read position limits as the real position limits
                    self.motor_limits[name] = (dxl_min_position, dxl_max_position)
                    volts = self._read_voltage(motor_id)
                    volt_str = f'{volts:.1f} V' if volts is not None else 'unknown'
                    self.get_logger().info(
                        f'Initialized motor {name} (ID: {motor_id}), '
                        f'limits: {dxl_min_position}-{dxl_max_position}, voltage: {volt_str}'
                    )
                    if volts is not None and volts < self.VOLTAGE_WARN_THRESHOLD * 0.1:
                        self.get_logger().warn(
                            f'Motor {name} voltage {volt_str} is near the undervoltage trip point!'
                        )
                else:
                    self.get_logger().warn(f'Failed to initialize motor {name} (ID: {motor_id})')

                # Stagger motor enables to reduce simultaneous inrush current
                time.sleep(0.05)
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

                # Check Hardware Error Status (address 50). Non-zero = fault.
                dxl_hw_error_status, dxl_comm_result, _ = self.packet_handler.read1ByteTxRx(
                    self.port_handler, motor_id, self.ADDR_HW_ERROR_STATUS
                )
                if dxl_comm_result != COMM_SUCCESS or dxl_hw_error_status != 0:
                    is_voltage = bool(dxl_hw_error_status & self.HW_ERROR_VOLTAGE_BIT)
                    label = 'undervoltage' if is_voltage else f'hw error 0x{dxl_hw_error_status:02x}'
                    volts = self._read_voltage(motor_id)
                    volt_str = f' ({volts:.1f} V)' if volts is not None else ''
                    self.get_logger().warn(
                        f'Motor {name} (ID: {motor_id}) {label}{volt_str} — attempting recovery'
                    )
                    if not self._recover_motor(motor_id, name):
                        continue  # recovery failed, skip this motor but keep going
                    # Fall through and execute the command after successful recovery

                # Check Present Load (address 41, 2 bytes). Bits 0-9 = magnitude (0-1023),
                # bit 10 = direction. Overload if magnitude > threshold.
                dxl_present_load, dxl_comm_result, _ = self.packet_handler.read2ByteTxRx(
                    self.port_handler, motor_id, 41
                )
                load_magnitude = dxl_present_load & 0x3FF  # strip direction bit
                if dxl_comm_result != COMM_SUCCESS or load_magnitude > 700:
                    self.get_logger().warn(f'Motor {motor_id} overloaded (load={load_magnitude}), skipping command')
                    continue

                # Apply per-joint speed from velocity field before commanding position.
                # The sequence player calculates MOVING_SPEED from distance/duration so the
                # motor physically moves at the right rate (0 = keep current speed).
                if i < len(msg.velocity) and msg.velocity[i] > 0:
                    speed = int(msg.velocity[i])
                    self.packet_handler.write2ByteTxRx(
                        self.port_handler, motor_id, self.ADDR_MOVING_SPEED, speed
                    )

                # Convert position to motor units (0-1023 for XL-320)
                motor_position = self.position_to_motor_units(name, position)

                # Send command to motor
                self.set_motor_position(motor_id, motor_position)
                self.current_positions[name] = motor_position
                time.sleep(0.003)  # 3 ms stagger prevents simultaneous current spikes

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
            controller_pos = self.motor_units_to_controller(motor_position)
            msg.position.append(controller_pos)
            msg.velocity.append(0.0)  # TODO: Read actual velocity
            msg.effort.append(0.0)    # TODO: Read actual effort

        self.joint_state_pub.publish(msg)

    def motor_units_to_controller(self, motor_position: int) -> float:
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
        
    def list_joint_limits_callback(self, _, response):
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
