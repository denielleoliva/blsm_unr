#!/usr/bin/env python3
from dynamixel_sdk import *  # pip install dynamixel-sdk
import time

#-----------------------------------------------------------
# User settings – change these for your setup
#-----------------------------------------------------------
DEVICENAME = "/dev/ttyUSB0"    # e.g. Windows: "COM3"
BAUDRATE   = 1000000
PROTOCOL_VERSION = 2.0

DXL_IDS = [3]  # Only motor 1 detected - check IDs for motors 2, 3, 4

# Control table addresses for XL-320 (NOT XL-330!)
ADDR_TORQUE_ENABLE = 24
ADDR_GOAL_POSITION = 30
LEN_GOAL_POSITION  = 2         # 2 bytes for XL-320

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0

#-----------------------------------------------------------
# Position limits for XL-320: 0–1023
#-----------------------------------------------------------
POS_A = [200]
POS_B = [800]

#-----------------------------------------------------------
# Helper: convert integer position to 2-byte little-endian list for XL-320
#-----------------------------------------------------------
def int_to_2bytes(value):
    return [
        DXL_LOBYTE(value),
        DXL_HIBYTE(value),
    ]

#-----------------------------------------------------------
# Main
#-----------------------------------------------------------
if __name__ == "__main__":
    # Open port
    port_handler = PortHandler(DEVICENAME)
    if not port_handler.openPort():
        print("❌ Failed to open the port")
        quit()

    # Set baudrate
    if not port_handler.setBaudRate(BAUDRATE):
        print("❌ Failed to set baudrate")
        quit()

    packet_handler = PacketHandler(PROTOCOL_VERSION)

    # Enable torque on all servos
    for dxl_id in DXL_IDS:
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Comm error on ID {dxl_id}:", packet_handler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print(f"Status error on ID {dxl_id}:", packet_handler.getRxPacketError(dxl_error))

    # Create a Sync Write handler for goal position
    group_sync_write = GroupSyncWrite(
        port_handler,
        packet_handler,
        ADDR_GOAL_POSITION,
        LEN_GOAL_POSITION
    )

    def move_all(goal_positions):
        """Send one sync-write packet so all motors move at once."""
        group_sync_write.clearParam()

        for dxl_id, pos in zip(DXL_IDS, goal_positions):
            # Clamp to [0, 1023] just in case
            pos_clamped = max(0, min(1023, pos))
            param_goal = int_to_2bytes(pos_clamped)
            add_ok = group_sync_write.addParam(dxl_id, param_goal)
            if not add_ok:
                print(f"❌ Failed to add param for ID {dxl_id}")
        
        dxl_comm_result = group_sync_write.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("❌ SyncWrite error:", packet_handler.getTxRxResult(dxl_comm_result))

    print("➡ Moving to POS_A...")
    move_all(POS_A)
    time.sleep(2.0)

    print("➡ Moving to POS_B...")
    move_all(POS_B)
    time.sleep(2.0)

    # Disable torque before exiting
    for dxl_id in DXL_IDS:
        packet_handler.write1ByteTxRx(
            port_handler, dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )

    port_handler.closePort()
    print("✅ Done.")
