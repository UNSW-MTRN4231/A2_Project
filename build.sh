#!/bin/bash
rm -rf build/
rm -rf install/
rm -rf log/

# Build yolov8_msgs
colcon build --packages-select yolov8_msgs yolov8_ros yolov8_bringup
source install/setup.bash

# Build all other packages
colcon build --packages-skip yolov8_msgs yolov8_ros yolov8_bringup
source install/setup.bash
