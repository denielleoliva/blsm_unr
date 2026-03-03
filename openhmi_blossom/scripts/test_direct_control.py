#!/usr/bin/env python3
"""
Simple test script to directly control motors without conversions.
This helps diagnose if the issue is with our position conversion logic.
XL-320 motors: Position range 0-1023, Protocol 2.0
"""

from dynamixel_sdk import *
import time

# Settings - match what works in Dynamixel Wizard
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000  # XL-320 default is 1000000, but can be changed
PROTOCOL_VERSION = 2.0  # XL-320 uses Protocol 2.0

# Protocol 2.0 addresses for XL-320 (different from other XL series!)
ADDR_TORQUE_ENABLE = 24
ADDR_CW_ANGLE_LIMIT = 6
ADDR_CCW_ANGLE_LIMIT = 8
ADDR_GOAL_POSITION = 30
ADDR_MOVING_SPEED = 32
ADDR_PRESENT_POSITION = 37
ADDR_MOVING = 49
ADDR_HARDWARE_ERROR_STATUS = 50

# Motor IDs
MOTOR_IDS = [1, 2, 3, 4]

# XL-320 position range: 0-1023 (NOT 0-4095!)
POSITION_MIN = 0
POSITION_MAX = 1023
POSITION_CENTER = 512

# XL-320 uses 2-byte position values even with Protocol 2.0

def test_motors():
    """Test direct motor control."""

    # Initialize
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if not port_handler.openPort():
        print(f"Failed to open port {DEVICENAME}")
        return False
    print(f"Opened port {DEVICENAME}")

    # Set baudrate
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"Failed to set baudrate to {BAUDRATE}")
        return False
    print(f"Set baudrate to {BAUDRATE}")
    print()

    # Test each motor
    for motor_id in MOTOR_IDS:
        print(f"Testing Motor {motor_id}:")

        # Ping motor first to check if it's responding
        model_number, result, error = packet_handler.ping(port_handler, motor_id)
        if result != COMM_SUCCESS:
            print(f"  ✗ Motor {motor_id} not responding: {packet_handler.getTxRxResult(result)}")
            print(f"     Check: motor ID, power, connections")
            print()
            continue
        else:
            print(f"  ✓ Motor {motor_id} is responding (Model: {model_number})")

        # Check hardware error status
        hw_error, result, error = packet_handler.read1ByteTxRx(
            port_handler, motor_id, ADDR_HARDWARE_ERROR_STATUS
        )
        if result == COMM_SUCCESS and hw_error != 0:
            print(f"  ⚠ Hardware Error Status: {hw_error} (0x{hw_error:02X})")
            error_bits = {
                0x01: "Input Voltage Error",
                0x04: "Overheating Error",
                0x08: "Motor Encoder Error",
                0x10: "Electrical Shock Error",
                0x20: "Overload Error"
            }
            for bit, desc in error_bits.items():
                if hw_error & bit:
                    print(f"     - {desc}")
        elif result == COMM_SUCCESS:
            print(f"  ✓ No hardware errors")

        # Read angle limits
        cw_limit, result, error = packet_handler.read2ByteTxRx(
            port_handler, motor_id, ADDR_CW_ANGLE_LIMIT
        )
        ccw_limit, result, error = packet_handler.read2ByteTxRx(
            port_handler, motor_id, ADDR_CCW_ANGLE_LIMIT
        )
        if result == COMM_SUCCESS:
            print(f"  Angle limits: CW={cw_limit}, CCW={ccw_limit}")
            if cw_limit == ccw_limit:
                print(f"  ⚠ Warning: Wheel mode enabled (both limits are {cw_limit})")

        # Read current position (2 bytes for XL-320)
        try:
            present_pos, result, error = packet_handler.read2ByteTxRx(
                port_handler, motor_id, ADDR_PRESENT_POSITION
            )

            if result == COMM_SUCCESS:
                print(f"  Current position: {present_pos}")
            else:
                print(f"  Failed to read position: {packet_handler.getTxRxResult(result)}")
                continue
        except IndexError:
            print(f"  ✗ Communication error - motor may not be responding properly")
            continue

        # Set moving speed (if it's 0, motor won't move!)
        result, error = packet_handler.write2ByteTxRx(
            port_handler, motor_id, ADDR_MOVING_SPEED, 200
        )
        if result == COMM_SUCCESS:
            print(f"  Set moving speed to 200")
        else:
            print(f"  Warning: Failed to set moving speed")

        # Enable torque
        result, error = packet_handler.write1ByteTxRx(
            port_handler, motor_id, ADDR_TORQUE_ENABLE, 1
        )

        if result != COMM_SUCCESS:
            print(f"  Failed to enable torque: {packet_handler.getTxRxResult(result)}")
            continue
        print(f"  ✓ Torque enabled")

        # Test movement: move to center position (512 for XL-320)
        target_pos = POSITION_CENTER
        print(f"  Moving to position {target_pos} (center)...")

        result, error = packet_handler.write2ByteTxRx(
            port_handler, motor_id, ADDR_GOAL_POSITION, target_pos
        )

        if result == COMM_SUCCESS:
            print(f"  Command sent successfully")
            if error != 0:
                print(f"  Motor error: {packet_handler.getRxPacketError(error)}")
        else:
            print(f"  Failed to write position: {packet_handler.getTxRxResult(result)}")

        # Check if motor is moving
        time.sleep(0.2)
        is_moving, result, error = packet_handler.read1ByteTxRx(
            port_handler, motor_id, ADDR_MOVING
        )
        if result == COMM_SUCCESS:
            print(f"  Moving status: {'Yes' if is_moving else 'No'}")

        # Wait for movement
        time.sleep(1.5)

        # Read new position (2 bytes for XL-320)
        new_pos, result, error = packet_handler.read2ByteTxRx(
            port_handler, motor_id, ADDR_PRESENT_POSITION
        )

        if result == COMM_SUCCESS:
            print(f"  New position: {new_pos}")
            diff = abs(new_pos - target_pos)
            if diff < 20:
                print(f"  ✓ Motor moved successfully!")
            else:
                print(f"  ✗ Motor did not reach target (diff: {diff})")
                if new_pos == present_pos:
                    print(f"  ⚠ Position unchanged - motor did not move at all")

        print()

    # Close port
    port_handler.closePort()
    print("Test complete")

    return True

if __name__ == '__main__':
    print("Direct Motor Control Test")
    print("=" * 50)
    print()
    test_motors()
