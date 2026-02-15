#!/usr/bin/env python3
"""
Utility script to read and clear Dynamixel motor hardware errors.
Run this when motors report hardware error status.
"""

from dynamixel_sdk import *
import sys

# Protocol 2.0 addresses
ADDR_TORQUE_ENABLE = 64
ADDR_HARDWARE_ERROR_STATUS = 70

# Connection settings
DEVICENAME = '/dev/ttyUSB0'
BAUDRATE = 1000000
PROTOCOL_VERSION = 2.0

# Motor IDs
MOTOR_IDS = [1, 2, 3, 4]  # lazy_susan, motor_front, motor_back_left, motor_back_right

def clear_errors():
    """Read and clear hardware errors on all motors."""

    # Initialize PortHandler and PacketHandler
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

    # Check each motor
    for motor_id in MOTOR_IDS:
        print(f"Motor {motor_id}:")

        # First disable torque
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, motor_id, ADDR_TORQUE_ENABLE, 0
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(f"  Failed to disable torque: {packet_handler.getTxRxResult(dxl_comm_result)}")
            continue

        # Read hardware error status
        error_status, dxl_comm_result, dxl_error = packet_handler.read1ByteTxRx(
            port_handler, motor_id, ADDR_HARDWARE_ERROR_STATUS
        )

        if dxl_comm_result != COMM_SUCCESS:
            print(f"  Failed to read error status: {packet_handler.getTxRxResult(dxl_comm_result)}")
            continue

        # Decode error bits
        errors = []
        if error_status & 0x01:
            errors.append("Input Voltage Error")
        if error_status & 0x04:
            errors.append("Overheating Error")
        if error_status & 0x08:
            errors.append("Motor Encoder Error")
        if error_status & 0x10:
            errors.append("Electrical Shock Error")
        if error_status & 0x20:
            errors.append("Overload Error")

        if errors:
            print(f"  Error status: {error_status} (0x{error_status:02X})")
            for error in errors:
                print(f"    - {error}")
        else:
            print(f"  No errors (status: {error_status})")

        # The hardware error status register is read-only and clears when the issue is resolved
        # We can try to reboot the motor which will clear transient errors
        print(f"  Rebooting motor...")

        # Reboot command (0x08)
        dxl_comm_result, dxl_error = packet_handler.reboot(port_handler, motor_id)

        if dxl_comm_result == COMM_SUCCESS:
            print(f"  Reboot successful")
        else:
            print(f"  Reboot failed: {packet_handler.getTxRxResult(dxl_comm_result)}")

        print()

    # Close port
    port_handler.closePort()
    print("Port closed")

    return True

if __name__ == '__main__':
    print("Dynamixel Hardware Error Checker and Clearer")
    print("=" * 50)
    print()

    clear_errors()

    print()
    print("If errors persist:")
    print("  - Check motor connections and power supply")
    print("  - Ensure motors are not mechanically blocked")
    print("  - Verify power supply voltage is correct (typically 12V)")
    print("  - Check for overheating - allow motors to cool if needed")
