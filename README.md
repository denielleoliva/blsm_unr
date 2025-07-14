# XL330 ROS 2 Control

This package provides a full ROS 2 `ros2_control` integration for controlling 4 Dynamixel XL-330 motors using a USB2Dynamixel converter.

## Installation

```bash
sudo apt install ros-humble-ros2-control ros-humble-ros2-controllers \
                 ros-humble-joint-state-publisher ros-humble-controller-manager \
                 ros-humble-xacro

pip install dynamixel-sdk
```

Clone and build the repo:

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
# clone this package
cd ~/ros2_ws
colcon build --packages-select xl330_ros2_control
source install/setup.bash
```

## Usage

To launch the full robot with 4-motor support:

```bash
ros2 launch xl330_ros2_control bringup.launch.py
```

To teleoperate using the keyboard:

```bash
ros2 run xl330_ros2_control teleop_xl330.py
```

- Use keys `1`–`4` to select a joint
- Use `a` and `d` to move the selected joint
- Press `q` to quit

## Motor Setup

Ensure:
- All motors are set to Protocol 2.0
- Each XL-330 has a unique ID: 1 (base), 2 (left), 3 (right), 4 (back)
- Motors are connected via a USB2Dynamixel TTL adapter (e.g. U2D2)

## Calibration

1. Start the system.
2. Use teleop keys to set each joint to a neutral/home position.
3. Record the angles (in radians) for each joint at rest.
4. Update initial values in your controller config or enforce offsets in your hardware interface if needed.
5. Optionally: use Dynamixel Wizard 2.0 to visually center servo limits.