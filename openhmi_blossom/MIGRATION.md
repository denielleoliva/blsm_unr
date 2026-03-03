# Migration Guide: OpenHMI/Blossom to ROS 2

This guide helps you migrate from the original Python-based Blossom control system to the ROS 2 package.

## Overview of Changes

### Architecture

**Original System:**
- Standalone Python application
- Web-based UI (Flask server)
- Direct serial communication
- JSON-based sequences
- Mobile app control

**ROS 2 System:**
- Distributed node architecture
- ROS 2 topics/services/actions
- Parameterized configuration
- YAML-based sequences
- Standard ROS interfaces

## Step-by-Step Migration

### 1. Hardware Setup

No changes needed - the ROS 2 package uses the same hardware:
- 4 Dynamixel motors
- USB2Dynamixel adapter or OpenCM board
- Same power requirements

### 2. Installing ROS 2 Package

```bash
# Install ROS 2 (if not already installed)
# Follow: https://docs.ros.org/en/humble/Installation.html

# Install the package
cd ~/ros2_ws/src
git clone <repository_url> openhmi_blossom
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --packages-select openhmi_blossom
source install/setup.bash
```

### 3. Converting Sequences

#### Original Format (JSON)
```json
{
  "yes": {
    "frames": [
      {
        "positions": [2048, 2048, 1800, 1800],
        "duration": 300
      },
      {
        "positions": [2048, 2048, 2300, 2300],
        "duration": 300
      }
    ]
  }
}
```

#### ROS 2 Format (YAML)
```yaml
yes:
  keyframes:
    - joints:
        tower_1: 0.5    # 2048/4095 ≈ 0.5
        tower_2: 0.5
        tower_3: 0.44   # 1800/4095 ≈ 0.44
        tower_4: 0.44
      duration: 0.3     # 300ms = 0.3s
    - joints:
        tower_1: 0.5
        tower_2: 0.5
        tower_3: 0.56   # 2300/4095 ≈ 0.56
        tower_4: 0.56
      duration: 0.3
```

#### Conversion Script

```python
#!/usr/bin/env python3
import json
import yaml

def convert_sequence(json_seq):
    """Convert JSON sequence to YAML format."""
    yaml_seq = {'keyframes': []}
    
    for frame in json_seq['frames']:
        keyframe = {
            'joints': {
                'tower_1': frame['positions'][0] / 4095.0,
                'tower_2': frame['positions'][1] / 4095.0,
                'tower_3': frame['positions'][2] / 4095.0,
                'tower_4': frame['positions'][3] / 4095.0,
            },
            'duration': frame['duration'] / 1000.0
        }
        yaml_seq['keyframes'].append(keyframe)
    
    return yaml_seq

# Usage
with open('original_sequences.json', 'r') as f:
    original = json.load(f)

converted = {}
for name, seq in original.items():
    converted[name] = convert_sequence(seq)

with open('converted_sequences.yaml', 'w') as f:
    yaml.dump(converted, f, default_flow_style=False)
```

### 4. Replacing Control Code

#### Original: Start Robot
```python
# Original
python start.py -p /dev/ttyACM0
```

#### ROS 2: Launch Robot
```bash
# ROS 2
ros2 launch openhmi_blossom blossom.launch.py port:=/dev/ttyACM0
```

#### Original: Play Sequence
```python
# Original Python code
from blossom import Blossom

robot = Blossom()
robot.do_sequence('yes')
```

#### ROS 2: Play Sequence
```python
# ROS 2 Python node
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class BlossomClient(Node):
    def __init__(self):
        super().__init__('blossom_client')
        self.pub = self.create_publisher(String, 'play_sequence', 10)
    
    def play_sequence(self, name):
        msg = String()
        msg.data = name
        self.pub.publish(msg)

rclpy.init()
client = BlossomClient()
client.play_sequence('yes')
```

Or from command line:
```bash
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'yes'" --once
```

### 5. Web Interface Integration

The original system had a built-in web interface. For ROS 2:

#### Option 1: ROS 2 Web Bridge
```bash
# Install ros2-web-bridge
sudo apt install ros-humble-rosbridge-suite

# Launch
ros2 launch rosbridge_server rosbridge_websocket_launch.xml
```

#### Option 2: Custom Web Interface
Create a web interface that publishes to ROS 2 topics:

```javascript
// Using roslibjs
var ros = new ROSLIB.Ros({
  url : 'ws://localhost:9090'
});

var sequenceTopic = new ROSLIB.Topic({
  ros : ros,
  name : '/play_sequence',
  messageType : 'std_msgs/String'
});

function playSequence(name) {
  var msg = new ROSLIB.Message({
    data : name
  });
  sequenceTopic.publish(msg);
}
```

### 6. Mobile App Integration

#### Original: Mobile App
The original system used a mobile app for direct control.

#### ROS 2: Multiple Options

**Option 1: ROS 2 Mobile App**
- Use existing ROS mobile apps (ROS Control, RViz AR)
- Publish to `/target_pose` topic

**Option 2: Custom App with ROS Bridge**
```kotlin
// Android example with ros2_android
import org.ros2.android.core.*

val node = Node("mobile_controller")
val publisher = node.createPublisher("play_sequence", String)

fun playGesture(name: String) {
    val msg = String()
    msg.data = name
    publisher.publish(msg)
}
```

### 7. Camera Integration

#### Original: Face Tracking
```python
# Original system
import cv2
# Direct integration with Blossom

def track_face():
    # Face detection code
    # Direct motor commands
```

#### ROS 2: Face Tracking Node
```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
import cv2
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

class FaceTracker(Node):
    def __init__(self):
        super().__init__('face_tracker')
        self.bridge = CvBridge()
        
        # Subscribe to camera
        self.image_sub = self.create_subscription(
            Image, '/camera/image_raw',
            self.image_callback, 10)
        
        # Publish target pose
        self.pose_pub = self.create_publisher(
            Pose, '/target_pose', 10)
    
    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Face detection
        # ... OpenCV code ...
        
        # Publish target pose
        pose = Pose()
        pose.orientation.y = pitch
        pose.orientation.z = yaw
        self.pose_pub.publish(pose)
```

### 8. Configuration Management

#### Original: Hard-coded Config
```python
# Original: config in Python files
MOTOR_IDS = {'tower1': 1, 'tower2': 2, ...}
PORT = '/dev/ttyACM0'
```

#### ROS 2: Parameter Files
```yaml
# config/motor_config.yaml
motor_ids:
  tower_1: 1
  tower_2: 2
  tower_3: 3
  tower_4: 4

motor_limits:
  tower_1: [0, 4095]
  # ...
```

Load at runtime:
```bash
ros2 launch openhmi_blossom blossom.launch.py \
  motor_config:=/path/to/custom_config.yaml
```

### 9. Debugging and Monitoring

#### Original System
```python
# Print statements
print("Playing sequence:", name)
```

#### ROS 2 Tools
```bash
# View all topics
ros2 topic list

# Monitor joint states
ros2 topic echo /joint_states

# Monitor sequence status
ros2 topic echo /sequence_status

# View node graph
rqt_graph

# Record data
ros2 bag record -a

# Replay data
ros2 bag play <bagfile>
```

### 10. Testing

#### Original
```python
# Manual testing via CLI or web UI
```

#### ROS 2
```bash
# Unit tests
colcon test --packages-select openhmi_blossom

# Integration tests
ros2 launch openhmi_blossom test_gestures.launch.py

# Record and replay
ros2 bag record /joint_commands
ros2 bag play <bagfile>
```

## Feature Comparison

| Feature | Original | ROS 2 |
|---------|----------|-------|
| Motor Control | Direct serial | Via ROS topics |
| Sequences | JSON files | YAML files |
| Web UI | Built-in Flask | Via rosbridge |
| Mobile App | Custom app | ROS mobile apps |
| Face Tracking | Direct integration | Separate node |
| Configuration | Hard-coded | Parameters/YAML |
| Logging | Print statements | ROS logging |
| Monitoring | Custom tools | rqt, rviz |
| Recording | Custom | rosbag |
| Simulation | None | Gazebo ready |

## Benefits of ROS 2

1. **Modularity**: Separate nodes for different functions
2. **Reusability**: Use existing ROS packages (cameras, perception, etc.)
3. **Scalability**: Easy to add new capabilities
4. **Debugging**: Rich tooling (rqt, rviz, rosbag)
5. **Standards**: Common interfaces across robots
6. **Community**: Large ecosystem of packages
7. **Multi-robot**: Easy to scale to multiple robots

## Common Pitfalls

### 1. Serial Port Permissions
```bash
# Always check permissions
ls -l /dev/ttyACM0

# Add to dialout group
sudo usermod -a -G dialout $USER
```

### 2. Sequence Format
- Original uses motor units (0-4095)
- ROS 2 uses normalized values (0.0-1.0)
- Remember to convert!

### 3. Timing
- Original uses milliseconds
- ROS 2 uses seconds
- Divide by 1000!

### 4. Motor IDs
- Must match your hardware configuration
- Check with Dynamixel Wizard if unsure

## Getting Help

1. Check ROS 2 logs:
```bash
ros2 launch openhmi_blossom blossom.launch.py --ros-args --log-level debug
```

2. Visualize system:
```bash
rqt_graph
```

3. Test components individually:
```bash
# Just motor interface
ros2 run openhmi_blossom motor_interface

# Just sequence player
ros2 run openhmi_blossom sequence_player
```

4. Community resources:
- ROS Answers: answers.ros.org
- ROS Discourse: discourse.ros.org
- Original Blossom: github.com/hrc2/blossom-public/issues

## Next Steps

After migration:
1. Test all gestures work correctly
2. Integrate with your perception system
3. Add custom behaviors as needed
4. Consider MoveIt 2 for advanced motion planning
5. Set up simulation in Gazebo
6. Create launch files for your specific use case

## Conclusion

While the migration requires some work, the ROS 2 system provides:
- Better integration with robotics ecosystem
- More flexible and modular architecture
- Professional-grade tooling and debugging
- Easier collaboration and code sharing
- Future-proof foundation for research

The initial time investment pays off in maintainability and extensibility!
