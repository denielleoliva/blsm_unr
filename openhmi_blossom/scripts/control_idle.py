#!/usr/bin/env python3
"""
Control idle animation for Blossom robot

Usage:
  python3 control_idle.py stop   # Stop idle animation
  python3 control_idle.py start  # Start idle animation
"""

import sys
import rclpy
from rclpy.node import Node
from std_msgs.msg import String


def control_idle(command):
    """Send idle control command"""
    rclpy.init()
    node = rclpy.create_node('idle_controller')
    pub = node.create_publisher(String, 'behavior_command', 10)
    
    # Wait for connection
    import time
    time.sleep(0.5)
    
    msg = String()
    if command == 'stop':
        msg.data = 'stop_idle'
        print("⏸ Stopping idle animation...")
    elif command == 'start':
        msg.data = 'start_idle'
        print("▶ Starting idle animation...")
    else:
        print(f"❌ Unknown command: {command}")
        print("Usage: python3 control_idle.py [stop|start]")
        node.destroy_node()
        rclpy.shutdown()
        return
    
    # Publish multiple times to ensure delivery
    for i in range(5):
        pub.publish(msg)
        time.sleep(0.1)
    
    print("✅ Command sent")
    
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    if len(sys.argv) < 2:
        print("Usage: python3 control_idle.py [stop|start]")
        print("\nExamples:")
        print("  python3 control_idle.py stop   # Stop idle animation")
        print("  python3 control_idle.py start  # Start idle animation")
        sys.exit(1)
    
    command = sys.argv[1].lower()
    control_idle(command)