#!/bin/bash

# Source the ROS2 environment
source install/setup.bash

# Launch the specified ROS2 launch file
ros2 launch bringup system_launch.py
