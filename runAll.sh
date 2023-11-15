source install/setup.bash

sleep 3

gnome-terminal -t "DriverServer" -e 'ros2 launch ur_robot_driver ur_control.launch.py ur_type:=ur5e robot_ip:=192.168.0.100 use_fake_hardware:=false launch_rviz:=false'

gnome-terminal -t "MoveitServer" -e 'ros2 launch ur_moveit_config ur_moveit.launch.py robot_ip:=192.168.0.100 ur_type:=ur5e launch_rviz:=true' 

gnome-terminal -t "YoloV8" -e 'ros2 launch yolov8_bringup yolov8.launch.py input_image_topic:='/image_warped' device:='cpu' model:=yolov8n-seg.pt'

sleep 10

cd launch/

ros2 launch A2.xml