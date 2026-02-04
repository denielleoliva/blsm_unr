#!/usr/bin/env python3
"""
Scan for Dynamixel motors at different baudrates and protocols.
This helps identify the correct settings when motors aren't responding.
"""

from dynamixel_sdk import *
import time

DEVICENAME = '/dev/ttyUSB1'

# Common baudrates for Dynamixel motors
BAUDRATES = [
    9600,
    57600,
    115200,
    1000000,
    2000000,
    3000000,
    4000000,
]

# XL-320 uses Protocol 2.0
PROTOCOL_VERSIONS = [2.0, 1.0]

# Scan IDs 1-10
MOTOR_IDS = range(1, 11)

def scan_motors():
    """Scan for motors at different baudrates and protocols."""

    print("Scanning for Dynamixel motors...")
    print("=" * 60)

    found_motors = []

    for protocol in PROTOCOL_VERSIONS:
        print(f"\nTrying Protocol {protocol}:")
        packet_handler = PacketHandler(protocol)

        for baudrate in BAUDRATES:
            print(f"  Baudrate {baudrate}...", end=" ", flush=True)

            port_handler = PortHandler(DEVICENAME)

            if not port_handler.openPort():
                print("Failed to open port!")
                continue

            if not port_handler.setBaudRate(baudrate):
                print("Failed to set baudrate!")
                port_handler.closePort()
                continue

            motors_at_this_config = []

            for motor_id in MOTOR_IDS:
                try:
                    model_number, result, error = packet_handler.ping(port_handler, motor_id)

                    if result == COMM_SUCCESS:
                        motors_at_this_config.append({
                            'id': motor_id,
                            'model': model_number,
                            'protocol': protocol,
                            'baudrate': baudrate
                        })
                except:
                    pass

            port_handler.closePort()

            if motors_at_this_config:
                print(f"Found {len(motors_at_this_config)} motor(s)!")
                for motor in motors_at_this_config:
                    print(f"    ID {motor['id']}: Model {motor['model']}")
                    found_motors.append(motor)
            else:
                print("No motors found")

    print("\n" + "=" * 60)
    print("SCAN COMPLETE")
    print("=" * 60)

    if found_motors:
        print(f"\nFound {len(found_motors)} motor(s) total:\n")
        for motor in found_motors:
            print(f"Motor ID {motor['id']}:")
            print(f"  Model Number: {motor['model']}")
            print(f"  Protocol: {motor['protocol']}")
            print(f"  Baudrate: {motor['baudrate']}")
            print()

        # Show recommended settings
        if found_motors:
            first = found_motors[0]
            print("Recommended settings for your scripts:")
            print(f"  PROTOCOL_VERSION = {first['protocol']}")
            print(f"  BAUDRATE = {first['baudrate']}")
            print(f"  MOTOR_IDS = {[m['id'] for m in found_motors]}")
    else:
        print("\nNo motors found!")
        print("Check:")
        print("  - Motor power supply is connected")
        print("  - USB cable is connected to correct port")
        print("  - Motor IDs (script only scans IDs 1-10)")

if __name__ == '__main__':
    scan_motors()
