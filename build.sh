#!/bin/bash

# Build yolov8_msgs
colcon build --packages-select yolov8_msgs
source install/setup.bash

# Build all other packages
colcon build --packages-skip yolov8_msgs
source install/setup.bash

# Launch the specified ROS2 launch file
ros2 launch bringup system_launch.py