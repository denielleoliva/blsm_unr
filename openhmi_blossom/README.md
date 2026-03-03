# OpenHMI Blossom ROS 2 Package

ROS 2 package for controlling the OpenHMI/Blossom robot, converted from the original Python-based control system.

## Overview

This package provides a ROS 2 interface for the Blossom robot, a handcrafted open-source tensile robot designed for human-robot interaction research. The package includes:

- Low-level motor control interface
- Gesture sequence playback
- High-level behavior control
- Action server for gesture execution

## Architecture

```
┌─────────────────────────┐
│  Blossom Controller     │  ← High-level behaviors
└───────────┬─────────────┘
            │
            ├─→ /target_pose (Pose)
            ├─→ /behavior_command (String)
            │
┌───────────┴─────────────┐
│   Sequence Player       │  ← Gesture sequences
└───────────┬─────────────┘
            │
            ├─→ /joint_commands (JointState)
            │
┌───────────┴─────────────┐
│   Motor Interface       │  ← Hardware interface
└───────────┬─────────────┘
            │
            └─→ Serial → Dynamixel Motors
```

## Installation

### Prerequisites

- ROS 2 (Humble, Iron, or Jazzy)
- Python 3.8+
- Dynamixel SDK (`pip install dynamixel-sdk`)
- PyYAML (`pip install pyyaml`)

### Build Instructions

1. Create a ROS 2 workspace:
```bash
mkdir -p ~/openhmi_ws/src
cd ~/openhmi_ws/src
```

2. Clone this repository:
```bash
git clone <repository_url> openhmi_blossom
```

3. Install dependencies:
```bash
cd ~/openhmi_ws
rosdep install --from-paths src --ignore-src -r -y
```

4. Install Python dependencies:
```bash
pip install dynamixel-sdk pyyaml
```

5. Build the workspace:
```bash
cd ~/openhmi_ws
colcon build --packages-select openhmi_blossom
source install/setup.bash
```

## Hardware Setup

### Motor Configuration

The Blossom robot uses 4 **Dynamixel XL-320** servo motors:
- Motor 1 (ID: 1) - Lazy Susan (base rotation)
- Motor 2 (ID: 2) - Front head plate
- Motor 3 (ID: 3) - Back-left head plate
- Motor 4 (ID: 4) - Back-right head plate

**XL-320 Specifications:**
- Protocol: 2.0
- Position Range: 0-1023 (10-bit resolution, not 12-bit)
- Baudrate: 1000000 (1 Mbps)
- Voltage: 6-8.4V
- Communication: TTL Half-Duplex

### Wiring

1. Connect XL-320 motors in daisy-chain configuration
2. Connect the USB2Dynamixel or U2D2 adapter to your computer
3. Ensure proper power supply (**6-8.4V** for XL-320, NOT 12V!)
4. Default port is usually `/dev/ttyUSB0` (not `/dev/ttyACM0`)

### Port Permissions

Grant permission to access the serial port:
```bash
sudo usermod -a -G dialout $USER
# Log out and back in for changes to take effect

# Or temporarily for /dev/ttyUSB0:
sudo chmod 666 /dev/ttyUSB0
```

## Usage

### Basic Launch

Launch the complete system:
```bash
ros2 launch openhmi_blossom blossom.launch.py
```

### Custom Configuration

Launch with custom parameters:
```bash
ros2 launch openhmi_blossom blossom.launch.py \
  port:=/dev/ttyUSB0 \
  baudrate:=1000000 \
  enable_idle:=false
```

### Playing Gestures

Play a pre-defined gesture sequence:
```bash
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'yes'" --once
```

Available gestures (see `config/sequences.yaml`):
- `yes` - Nodding gesture
- `no` - Head shaking
- `happy` - Happy motion
- `look_left` - Look to the left
- `look_right` - Look to the right
- `idle` - Breathing animation

### Behavior Commands

Send high-level behavior commands:
```bash
ros2 topic pub /behavior_command std_msgs/msg/String "data: 'nod'" --once
```

### Direct Pose Control

Control robot pose directly:
```bash
ros2 topic pub /target_pose geometry_msgs/msg/Pose "{
  position: {x: 0.0, y: 0.0, z: 0.5},
  orientation: {x: 0.0, y: 0.1, z: 0.0, w: 1.0}
}" --once
```

## Nodes

### motor_interface

Low-level interface to Dynamixel motors.

**Subscribed Topics:**
- `/joint_commands` (sensor_msgs/JointState) - Joint position commands

**Published Topics:**
- `/joint_states` (sensor_msgs/JointState) - Current joint states

**Parameters:**
- `port` (string, default: "/dev/ttyUSB0") - Serial port for XL-320 motors
- `baudrate` (int, default: 1000000) - Communication baudrate (1 Mbps for XL-320)
- `motor_config` (string) - Path to motor configuration file
- `publish_rate` (double, default: 50.0) - Joint state publishing rate (Hz)

### sequence_player

Plays pre-recorded gesture sequences.

**Subscribed Topics:**
- `/play_sequence` (std_msgs/String) - Sequence name to play

**Published Topics:**
- `/joint_commands` (sensor_msgs/JointState) - Joint position commands
- `/sequence_status` (std_msgs/String) - Playback status

**Parameters:**
- `sequences_dir` (string) - Directory containing sequence files
- `default_duration` (double, default: 1.0) - Default keyframe duration
- `interpolation_points` (int, default: 10) - Points for interpolation

### blossom_controller

High-level controller for coordinating behaviors.

**Subscribed Topics:**
- `/target_pose` (geometry_msgs/Pose) - Target orientation and height
- `/behavior_command` (std_msgs/String) - High-level behavior commands

**Published Topics:**
- `/joint_commands` (sensor_msgs/JointState) - Joint position commands
- `/play_sequence` (std_msgs/String) - Sequence playback commands

**Parameters:**
- `idle_sequence` (string, default: "idle") - Sequence for idle behavior
- `enable_idle` (bool, default: true) - Enable idle behavior
- `idle_interval` (double, default: 10.0) - Seconds between idle triggers

### gesture_server

Action server for gesture execution with feedback.

**Action Servers:**
- `/execute_gesture` (control_msgs/action/FollowJointTrajectory)

**Subscribed Topics:**
- `/sequence_status` (std_msgs/String) - Playback status

**Published Topics:**
- `/play_sequence` (std_msgs/String) - Sequence commands

## Configuration

### Motor Configuration

Edit `config/motor_config.yaml` to customize motor settings:

```yaml
motor_ids:
  lazy_susan: 1        # Base rotation motor
  motor_front: 2       # Front head plate motor
  motor_back_left: 3   # Back-left head plate motor
  motor_back_right: 4  # Back-right head plate motor

motor_limits:
  lazy_susan: [0, 1023]      # XL-320 full range
  motor_front: [0, 400]      # Head plate pull range
  motor_back_left: [0, 400]  # Head plate pull range
  motor_back_right: [0, 400] # Head plate pull range
```

### Creating Gesture Sequences

Create new gesture sequences in YAML format:

```yaml
my_gesture:
  keyframes:
    - joints:
        tower_1: 0.5
        tower_2: 0.5
        tower_3: 0.3
        tower_4: 0.3
      duration: 0.5
    - joints:
        tower_1: 0.6
        tower_2: 0.6
        tower_3: 0.4
        tower_4: 0.4
      duration: 0.5
```

Joint positions are normalized values (0.0 to 1.0) that are automatically converted to motor units.

## Integration with Original Blossom Code

### Migrating from Original Python Code

The original Blossom codebase uses:
- Direct serial communication
- Web-based control interface
- Mobile app control

This ROS 2 package provides:
- Standard ROS 2 interfaces
- Action servers for feedback
- Parameter-based configuration
- Launch file support

### Key Differences

| Original | ROS 2 Package |
|----------|---------------|
| `start.py` | `ros2 launch openhmi_blossom blossom.launch.py` |
| Web UI | ROS 2 topics/services |
| Direct serial commands | Motor interface node |
| JSON sequences | YAML sequences |

### Porting Sequences

Convert original Blossom sequences to ROS 2 format:

**Original (JSON):**
```json
{
  "yes": {
    "frames": [
      {"positions": [2048, 2048, 1800, 1800], "duration": 300},
      {"positions": [2048, 2048, 2300, 2300], "duration": 300}
    ]
  }
}
```

**ROS 2 (YAML):**
```yaml
yes:
  keyframes:
    - joints:
        tower_1: 0.5
        tower_2: 0.5
        tower_3: 0.44
        tower_4: 0.44
      duration: 0.3
    - joints:
        tower_1: 0.5
        tower_2: 0.5
        tower_3: 0.56
        tower_4: 0.56
      duration: 0.3
```

## Troubleshooting

### Cannot open serial port

```bash
# Check port exists (XL-320 usually shows as ttyUSB0)
ls -l /dev/ttyUSB*

# Check permissions
sudo chmod 666 /dev/ttyUSB0

# Add user to dialout group
sudo usermod -a -G dialout $USER
```

### Motors not responding

1. **Check power supply**: XL-320 requires 6-8.4V (NOT 12V!)
2. **Verify baudrate**: Should be 1000000 for XL-320
3. **Check Protocol**: XL-320 uses Protocol 2.0
4. **Test with Dynamixel Wizard**: Verify motors work before using ROS
5. **Scan for motors**: Run `scripts/scan_motors.py` to find motor IDs
6. **Verify position range**: XL-320 uses 0-1023, not 0-4095

### Sequences not playing

1. Verify sequences_dir parameter points to valid directory
2. Check YAML syntax in sequence files
3. View logs: `ros2 topic echo /sequence_status`

## Advanced Usage

### Using with MoveIt 2

This package can be integrated with MoveIt 2 for motion planning:

1. Create URDF/XACRO model of Blossom
2. Configure MoveIt 2 with the motor interface
3. Use MoveIt 2 planning capabilities

### Integration with RealSense Camera

For face tracking and following:

```bash
# Launch RealSense camera
ros2 launch realsense2_camera rs_launch.py

# Your face tracking node
ros2 run your_package face_tracker

# Blossom will follow faces via /target_pose topic
```

### Custom Behaviors

Implement custom behavior nodes:

```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class CustomBehavior(Node):
    def __init__(self):
        super().__init__('custom_behavior')
        self.pub = self.create_publisher(String, 'behavior_command', 10)
        
    def wave_hello(self):
        msg = String()
        msg.data = 'wave'
        self.pub.publish(msg)
```

## Citation

If you use this ROS 2 package, please cite both the original Blossom paper and this package:

```bibtex
@article{suguitan2019blossom,
  title={Blossom: A Handcrafted Open-Source Robot},
  author={Suguitan, Michael and Hoffman, Guy},
  journal={ACM Transactions on Human-Robot Interaction (THRI)},
  volume={8},
  number={1},
  pages={1--27},
  year={2019},
  publisher={ACM}
}
```

## Contributing

Contributions are welcome! Please:

1. Fork the repository
2. Create a feature branch
3. Test your changes thoroughly
4. Submit a pull request

## License

This package is released under the MIT License. See LICENSE file for details.

## Support

For issues and questions:
- GitHub Issues: <repository_url>/issues
- Original Blossom: https://github.com/hrc2/blossom-public
- OpenHMI: https://github.com/interaction-lab/OpenHMI

## Acknowledgments

- Original Blossom robot by Cornell HRC2 Lab
- OpenHMI platform by USC Interaction Lab
- ROS 2 community

## Roadmap

- [ ] Dynamixel SDK integration
- [ ] RViz visualization support
- [ ] MoveIt 2 integration
- [ ] Gazebo simulation
- [ ] Web interface bridge
- [ ] Mobile app integration
- [ ] Face tracking integration
- [ ] Speech synthesis integration
