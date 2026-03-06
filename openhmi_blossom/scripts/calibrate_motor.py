#!/usr/bin/env python3
"""
XL-320 Motor Calibration Script for Blossom

Scans for a single motor, optionally reassigns its ID, then steps through
calibration positions so you can attach the horn and set string length.

Usage:
    python3 calibrate_motor.py [port]          # default port: /dev/ttyUSB0
"""

import sys
import time
from dynamixel_sdk import (
    PortHandler, PacketHandler, COMM_SUCCESS
)

# XL-320 settings
PROTOCOL  = 2.0
BAUDRATE  = 1_000_000
PORT      = sys.argv[1] if len(sys.argv) > 1 else '/dev/ttyUSB0'
SCAN_IDS  = range(1, 21)

# XL-320 control table
ADDR_ID            = 3
ADDR_TORQUE_ENABLE = 24
ADDR_MOVING_SPEED  = 32
ADDR_GOAL_POSITION = 30

CALIBRATION_SPEED  = 150  # slow enough to see movement clearly


def open_port(port: str):
    ph = PortHandler(port)
    pkt = PacketHandler(PROTOCOL)
    if not ph.openPort():
        sys.exit(f'Cannot open port {port}')
    if not ph.setBaudRate(BAUDRATE):
        sys.exit(f'Cannot set baudrate {BAUDRATE}')
    return ph, pkt


def scan(ph, pkt) -> list[int]:
    found = []
    print(f'Scanning IDs {SCAN_IDS.start}–{SCAN_IDS.stop - 1} on {PORT} …')
    for mid in SCAN_IDS:
        _, result, _ = pkt.ping(ph, mid)
        if result == COMM_SUCCESS:
            found.append(mid)
    return found


def set_torque(ph, pkt, mid: int, enable: bool):
    pkt.write1ByteTxRx(ph, mid, ADDR_TORQUE_ENABLE, 1 if enable else 0)


def set_speed(ph, pkt, mid: int, speed: int):
    pkt.write2ByteTxRx(ph, mid, ADDR_MOVING_SPEED, speed)


def go_to(ph, pkt, mid: int, position: int):
    pkt.write2ByteTxRx(ph, mid, ADDR_GOAL_POSITION, position)


def change_id(ph, pkt, old_id: int, new_id: int) -> bool:
    """Change motor ID. Torque must be disabled first."""
    set_torque(ph, pkt, old_id, False)
    time.sleep(0.1)
    result, error = pkt.write1ByteTxRx(ph, old_id, ADDR_ID, new_id)
    time.sleep(0.2)
    if result != COMM_SUCCESS or error != 0:
        return False
    _, ping_result, _ = pkt.ping(ph, new_id)
    return ping_result == COMM_SUCCESS


def main():
    ph, pkt = open_port(PORT)

    # ── Scan ──────────────────────────────────────────────────────────────────
    found = scan(ph, pkt)

    if len(found) == 0:
        sys.exit('No motors found. Is external power connected?')
    if len(found) > 1:
        sys.exit(f'Found {len(found)} motors ({found}). Connect only one motor at a time.')

    motor_id = found[0]
    print(f'Motor {motor_id} found!')

    # ── Optional ID change ────────────────────────────────────────────────────
    new_id_str = input(
        'Enter new motor ID (1–3 towers, 4 base, 5+ other). '
        'Press Enter to keep current ID: '
    ).strip()

    if new_id_str:
        new_id = int(new_id_str)
        print(f'Changing ID {motor_id} → {new_id} …', end=' ', flush=True)
        if change_id(ph, pkt, motor_id, new_id):
            motor_id = new_id
            print('OK')
        else:
            print('FAILED. Unplug and replug the motor, then run again.')
            ph.closePort()
            return

    # ── Enable torque and set calibration speed ───────────────────────────────
    set_torque(ph, pkt, motor_id, True)
    set_speed(ph, pkt, motor_id, CALIBRATION_SPEED)

    # ── Calibration steps ─────────────────────────────────────────────────────
    print('\n--- Calibration ---')

    go_to(ph, pkt, motor_id, 100)
    input('Position: 100 — Attach the horn, then press Enter.')

    go_to(ph, pkt, motor_id, 0)
    input('Position:   0 — Set string length, then press Enter.')

    go_to(ph, pkt, motor_id, 100)
    print('Position: 100 — Calibration complete!')

    ph.closePort()


if __name__ == '__main__':
    main()
