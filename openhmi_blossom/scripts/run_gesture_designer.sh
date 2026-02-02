#!/bin/bash
# Blossom Gesture Designer Launcher
# Fixes snap/system library conflicts

# Unset snap environment variables that cause conflicts
unset GTK_PATH
unset LD_LIBRARY_PATH
unset PYTHONPATH

# Use system libraries only
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:/lib/x86_64-linux-gnu

# Source ROS 2 if available
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi

if [ -f "$HOME/blossom_ws/install/setup.bash" ]; then
    source $HOME/blossom_ws/install/setup.bash
fi

# Launch with system Python (not snap)
/usr/bin/python3 "$(dirname "$0")/gesture_designer_gui.py" "$@"