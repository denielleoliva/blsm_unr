# Quick Start Guide

Get your Blossom robot running with ROS 2 in minutes!

## Prerequisites

- Ubuntu 22.04 or later
- ROS 2 Humble, Iron, or Jazzy installed
- Blossom robot assembled with Dynamixel motors
- USB connection to motors

## 5-Minute Setup

### 1. Install Package (2 minutes)

```bash
# Create workspace
mkdir -p ~/blossom_ws/src
cd ~/blossom_ws/src

# Clone repository
git clone <repository_url> openhmi_blossom

# Build
cd ~/blossom_ws
colcon build
source install/setup.bash
```

### 2. Set Permissions (1 minute)

```bash
# Add yourself to dialout group (requires logout/login)
sudo usermod -a -G dialout $USER

# OR temporarily:
sudo chmod 666 /dev/ttyACM0
```

### 3. Launch Robot (1 minute)

```bash
# Source your workspace
source ~/blossom_ws/install/setup.bash

# Launch the robot
ros2 launch openhmi_blossom blossom.launch.py
```

### 4. Test It! (1 minute)

```bash
# In a new terminal
source ~/blossom_ws/install/setup.bash

# Make the robot nod
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'yes'" --once

# Try other gestures
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'no'" --once
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'happy'" --once
```

## Common First-Time Issues

### Issue: "Cannot open /dev/ttyACM0"
**Solution:**
```bash
# Check what port your device is on
ls /dev/tty*

# If it's ttyUSB0 instead:
ros2 launch openhmi_blossom blossom.launch.py port:=/dev/ttyUSB0

# Fix permissions
sudo chmod 666 /dev/ttyACM0
```

### Issue: Motors not moving
**Solution:**
1. Check power supply to motors
2. Verify motors are connected
3. Test with Dynamixel Wizard first
4. Check motor IDs in config file

### Issue: "Package not found"
**Solution:**
```bash
# Re-source your workspace
cd ~/blossom_ws
source install/setup.bash

# Or add to bashrc
echo "source ~/blossom_ws/install/setup.bash" >> ~/.bashrc
```

## What's Next?

### Try Different Gestures

```bash
# Available gestures:
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'yes'" --once
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'no'" --once
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'happy'" --once
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'look_left'" --once
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'look_right'" --once
```

### Use the Example Client

```bash
cd ~/blossom_ws/src/openhmi_blossom/examples
python3 example_client.py
```

### Monitor the Robot

```bash
# See joint positions
ros2 topic echo /joint_states

# See what's happening
ros2 topic echo /sequence_status

# Visualize node graph
rqt_graph
```

### Create Your Own Gestures

1. Edit `config/sequences.yaml`
2. Add your gesture:
```yaml
my_gesture:
  keyframes:
    - joints:
        tower_1: 0.5
        tower_2: 0.5
        tower_3: 0.6
        tower_4: 0.6
      duration: 0.5
```
3. Reload and test:
```bash
ros2 topic pub /play_sequence std_msgs/msg/String "data: 'my_gesture'" --once
```

### Integrate with Your Code

Python:
```python
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MyController(Node):
    def __init__(self):
        super().__init__('my_controller')
        self.pub = self.create_publisher(String, 'play_sequence', 10)
    
    def greet(self):
        msg = String()
        msg.data = 'yes'
        self.pub.publish(msg)

rclpy.init()
controller = MyController()
controller.greet()
```

C++:
```cpp
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class MyController : public rclcpp::Node {
public:
    MyController() : Node("my_controller") {
        pub_ = create_publisher<std_msgs::msg::String>("play_sequence", 10);
    }
    
    void greet() {
        auto msg = std_msgs::msg::String();
        msg.data = "yes";
        pub_->publish(msg);
    }
    
private:
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
};
```

## Troubleshooting

### View Logs
```bash
ros2 launch openhmi_blossom blossom.launch.py --ros-args --log-level debug
```

### Test Individual Nodes
```bash
# Just motor interface
ros2 run openhmi_blossom motor_interface --ros-args -p port:=/dev/ttyACM0

# Just sequence player
ros2 run openhmi_blossom sequence_player
```

### Check System
```bash
# List all topics
ros2 topic list

# List all nodes
ros2 node list

# Check parameter values
ros2 param list /motor_interface
```

## Getting Help

- 📖 Full documentation: See `README.md`
- 🔄 Migration guide: See `MIGRATION.md`
- 💻 Examples: Check `examples/` directory
- 🐛 Issues: <repository_url>/issues
- 💬 Original Blossom: https://github.com/hrc2/blossom-public

## Tips for Success

1. **Start Simple**: Test basic gestures before complex behaviors
2. **Use Logs**: Add `--ros-args --log-level debug` when troubleshooting
3. **Visualize**: Use `rqt_graph` to understand data flow
4. **Save Configs**: Use launch files for your specific setup
5. **Test Hardware First**: Use Dynamixel Wizard to verify motors work

## What You Can Do Now

✅ Play pre-defined gestures
✅ Create custom gestures
✅ Control robot pose directly
✅ Monitor robot state
✅ Record and replay behaviors
✅ Integrate with your own code

## Next Steps

- Add face tracking with RealSense camera
- Integrate speech synthesis
- Add emotion recognition
- Create interactive behaviors
- Connect to other ROS packages
- Build custom applications

Happy robot-ing! 🤖
