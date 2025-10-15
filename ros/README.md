# Blossom ROS 2 Driver


This package enables control of a 3-motor Blossom robot using Dynamixel XL-330s and ROS 2 `ros2_control`. Includes real + fake hardware interfaces, launch files, idle behavior scripts, and URDF with a screen mount.


## Features
- 3 DOF: `base_joint`, `head_left`, `head_right`
- USB2Dynamixel XL-330 real hardware driver
- Simulated hardware fallback
- Breathing/idle behavior script
- Screen mount in URDF for face rendering
- ROS 2 launch integration with controllers


## Usage


```bash
ros2 launch blossom_ros2_driver bringup.launch.py
```


To teleoperate:
```bash
ros2 run blossom_ros2_driver teleop_blossom.py
```


To run idle animation:
```bash
ros2 run blossom_ros2_driver idle_breathing_behavior.py
```


## Dependencies
```bash
sudo apt install ros-humble-ros2-control ros-humble-joint-state-broadcaster \
ros-humble-controller-manager ros-humble-xacro
pip install dynamixel-sdk
```


---