# LAB 2 - TF2 - RVIZ - Moveit
The goal of this lab is to identify a red ball in 3d space and then move the end effector of the robot to the balls location. 
This lab is designed such that the different sections can and should be completed asynchronously.


Provided files:
 * Basic computer vision node which tracks the red ball
 * Stub packages and nodes for transformations and moveit 
 * Keyboard package for user system interactions
 * Setup for ur5e driver and moveit server

In this lab your team will:
 * Use a static transformation between 'world' and 'camera_frame'
 * Use a dynamic transformation between 'camera_frame' and 'ball_frame'
 * Display the transformation tree on rviz
 * Display a redball in 3d space on rviz 
 * Finish the collision objects in moveit
 * Identify the transformation between 'base_link' and 'ball_frame'
 * Move the robotic arm to the position of the red ball
 * Read in and activate functionality on keyboard inputs


Team task assignment (suggestion):
 * 1 member Transformations (Static, Listener)
 * 1 member Rviz,Transformation (Dyanmic, rviz)
 * 1 member Moveit, keyboard subscriber
 * 1 member Moveit, ur5e setup


## Connecting to the robotic arm

Connect to the control box using the orange ethernet cable.
Execute the setup script in 4231_scripts:

    ./setupRealur5e.sh

This will spawn two nodes and should open rviz and displaying the current configuration of the arm.
On the robotic arm teach tablet in manual mode create a program which only has the external control urcap program.
Run the program to establish communication between ros2 and the ur5e.



## Supplied Node - Ros2 keyboard
This is a utility node which allows a user to enter keys through a pygame terminal.
On keypress the is published as a string message to the topic 'keyboard_input'.
This node is supplied inside the '4231_utils' directory.

To run the node use the command:

    ros2 run util_keyboard util_keyboard 



## Supplied Node - Ros2 webcam interface
This will cover how to to stream the image from a webcam to a ros2 topic in an image message.
Make sure a webcam/usb camera is connected to ubuntu then execute the following command found in the 4231_scripts directory.

    ./camera.sh

Verify an image is publishing by opening RQT and adding the image viewer plugin
In some cases (in particular microsoft webcams) where the webcam is unable to produce an image run:

    qv4l2 -R -d /dev/video0

Then press the green arrow to activate the camera. 

You can verify video 0 is avalible by navigating to the /dev directory. Ie from a new terminal execute:

    cd ../..
    cd dev/
    ls



## Supplied Node - lab2_ball_estimation
*Note: this node is not perfect and may require some tweaking to improve performance*

This node generates a PoseStamped message and publishes it to the topic 'ball_pose' based on the estimated 3d position of the ball relative to the camera. 

To run the node:

    ros2 run lab2_ball_estimation lab2_ball_estimation 

The full message definition can be found at:

    https://docs.ros2.org/latest/api/geometry_msgs/msg/PoseStamped.html



## Transformations
TF2 Demo files can be found in 4231_demo_packages.
Stub files have been provided in src/lab2_transformation

### Static transformation

Fill out the provided stub file 'static_camera_transform.cpp' to generate a static frame between 'world' and 'camera_frame'

The transformation should be:

 * header.frame_id = "world"
 * child.frame_id = "camera_frame"
 * x = 1.2
 * y = 0.0
 * z = 1.0

The full message definition can be found at:

    https://docs.ros2.org/latest/api/geometry_msgs/msg/TransformStamped.html

Additionally the camera is facing down at ~45 degrees, you can use the following link to find the rotation variables. 

    https://quaternions.online/

Verify the static transformation works by running rviz and adding the tf2 topic, or generating a tf2 frame tree with tf2_tools. 

### Dynamic transformation

Fill out the provided stub file 'dynamic_ball_transform.cpp' to generate a dyanmic frame between 'camera_frame' and 'ball_frame'. This frame should broadcast everytime the ball estimation node publishes a PoseStamped message to the topic 'ball_pose'. 

The transformation should be:
 * header.frame_id = "camera_frame"
 * child.frame_id = "ball_frame"

The full message definition can be found at:

    https://docs.ros2.org/latest/api/geometry_msgs/msg/TransformStamped.html

Verify the dynamic transformation works by running the camera node, rviz and adding the tf2 topic, or generating a tf2 frame tree with tf2_tools. 



## Rviz
You should generate and display a red ball of size 0.2m at the location of the 'ball_frame' in rviz. 
This can either be done through the use of a custom node or by extending 'lab2_dynamic_transformation.cpp'.
Make sure to add the visualisation_msgs package to CMake and package.xml.

The full message definition can be found at:

    http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/Marker.html


An rviz demo can be found in 4231_demo_packages/src/demo_launch_param



## Moveit
This node moves the robotic arm to the position of the ball when the space key is pressed. 
A stub file can be found in the package lab2_moveit.

File can be launched using:

    ros2 launch lab2_moveit move_to_marker.launch.py 

*Note: a launch file is used to pass through the specific arm parameters, and doesn't affect the node*


### keyboard subscriber
Impliment a subscriber which subscribes to 'keyboard_input' and execute the robotic arm movement on a 'space' key press. 
'keyboard_input' is of type std_msgs/String and the full definition can be found below:

    https://docs.ros.org/en/latest/api/std_msgs/html/msg/String.html

### Transformation listener
Peform a transformation lookup which generates the transformation between 'base_link' and 'ball_frame'.
Save this transformation as a private variable so that it can be accessed when moveit plans and executes a movement. 
Refer to 4231_demo_packages/src/demo_tf2 or the humble tutorials:

https://docs.ros.org/en/humble/Tutorials/Intermediate/Tf2/Writing-A-Tf2-Listener-Cpp.html


### Collision Objects
While collision objects for the walls have been provided fill out a collision object for the table.
Use a height value of:
 * z = -0.03

### Plan and execute movement
Refer to 4231_demo_packages/src/demo_moveit_ur to plan and execute a movement. 



## Extensions
 * Use launch file to activate all nodes at the same time.
 * Seperate the moveit node into two seperate nodes, an 'arm_brain' and 'arm_execute' with a client service relationship.
 * Use moveit servo or constrain to move to the desired location.
 * Track the position of the ball in realtime using servo control.