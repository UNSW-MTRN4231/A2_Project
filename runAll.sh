source install/setup.bash

sleep 3

# DO NOT LAUNCH MOVEIT FROM HERE - BAD THINGS WILL HAPPEN

gnome-terminal -t "YoloV8" -e 'ros2 launch yolov8_bringup yolov8.launch.py input_image_topic:='/image_warped' device:='cpu' model:=yolov8n-seg.pt'

sleep 5

cd launch/

ros2 launch A2.xml