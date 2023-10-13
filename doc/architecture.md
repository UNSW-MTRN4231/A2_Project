# ROS2 Architecture Documentation

## Nodes
| Node | Description | Data from | Data to |
| - | - | - | - |
| usb_cam | Stream images from webcam |  | <ul><li>/image_raw |
| robot_state_broadcaster | Stream transformations of each robot joint | | <ul><li>/tool_tf |
| image_warper | Warp images such that they appear as if they were taken from an 'ideal' overhead perspective | <ul><li>/image_raw<li>/robot_state | <ul><li>/image_warped<li>/warp_pov_tf |
| yolov8_node | Use YOLOv8 model to segment 'pizza' and 'plate' in an image | <ul><li>/image_warped | <ul><li>/yolo_detections |
| plate_size_estimation | Estimate the plate size using a segmentation mask for 'plate' in the YOLO detections and the known transform of the image POV | <ul><li>/yolo_detections<li>/warped_pov_tf | <ul><li>/plate_size |
| pizza_size_estimation | Estimate the pizza size using a segmentation mask for 'pizza' in the YOLO detections and the known transform of the image POV | <ul><li>/yolo_detections<li>/warped_pov_tf | <ul><li>/pizza_size |
| plate_position_estimation | Estimate the position of the plate using the centroid of the segmentation mask for 'plate' in the YOLO detections and the known transform of the image POV | <ul><li>/yolo_detections<li>/warped_pov_tf | <ul><li>/plate_position |
| pizza_position_estimation | Estimate the position of the pizza using the centroid of the segmentation mask for 'pizza' in the YOLO detections and the known transform of the image POV | <ul><li>/yolo_detections<li>/warped_pov_tf | <ul><li>/pizza_position |
| pizza_rotation_estimation | Estimate the rotation of the pizza by applying SIFT to a cropped image of the pizza, and comparing with an initial reference image | <ul><li>/yolo_detections<li>/warped_pov_tf<li>\image_warped | <ul><li>/pizza_rotation |
| plate_tf_broadcaster | Broadcast the transformation of the plate | <ul><li>/plate_position | <ul><li>/plate_tf |
| pizza_tf_broadcaster | Broadcast the transformation of the pizza | <ul><li>/pizza_position<li>/pizza_rotation | <ul><li>/pizza_tf |
| markers | Broadcast marker messages for visualisation of the plate and pizza in Rviz | <ul><li>/plate_size<li>/pizza-size<li>/plate_tf<li>pizza_tf | <ul><li>TBD |
| detection_status_monitor | Publish timestamp upon successful detection of pizza and plate | <ul><li>/plate_tf<li>/pizza_tf | <ul><li>/operation_status |
| pizza_state_monitor | Monitor the number of slices removed from the pizza and publish the transform of the next pizza slice to be picked up | <ul><li>/operation_status<li>/pizza_size<li>/pizza_tf | <ul><li>/slice_tf |
| keyboard_input | Publish keystrokes | | <ul><li>/keyboard_input |
| high_level_control | Decide the next operation (tool pick/place, slice, or serve) based on keyboard input and completion of previous operations | <ul><li>/keyboard_input | <ul><li>/operation_command |
| moveit_servo | Execute servo control for precise positioning for the current operation | <ul><li>/operation_command | <ul><li>/operation_status |
| moveit_trajectory | Plan and execute a suitable trajectory for the current operation | <ul><li>/operation_command<li>/slice_tf | <ul><li>/operation_status |

## Topics
| Topic | Description | Message Type |
| - | - | - |
| /image_raw | Raw image from end effector webcam | sensor_msgs/Image.msg |
| /tool_tf | Transformation of the robot tool tip | geometry_msgs/msg/TransformStamped.msg |
| /image_warped | Webcam image warped such that it appears to be taken from an 'ideal' overhead POV | sensor_msgs/Image.msg |
| /warp_pov_tf | Tranformation of the 'ideal' overhead POV | geometry_msgs/msg/TransformStamped.msg |
| /yolo_detections | Masks of pizza and plate in input image | yolov8_msgs/msg/Mask.msg |
| /plate_size | Radius of the plate (mm) | std_msgs/Float32.msg |
| /pizza_size | Radius of the pizza (mm) | std_msgs/Float32.msg |
| /plate_position | Position of the centre of the plate (mm) | geometry_msgs/Point.msg |
| /pizza_position | Position of the centre of the pizza (mm) | geometry_msgs/Point.msg |
| /pizza_rotation | Rotation of the pizza from initial detection (rad) | std_msgs/Float32.msg |
| /plate_tf | Transformation of the plate | geometry_msgs/msg/TransformStamped.msg |
| /pizza_tf | Transformation of the pizza | geometry_msgs/msg/TransformStamped.msg |
| /slice_tf | Transformation of next slice to picked up (Actual points TBD) | geometry_msgs/msg/TransformStamped.msg |
| /keyboard_input | Keystrokes | keyboard/Key.msg |
| /operation_command | Next operation to be completed - valid operations are 'detect','slice','serve','pickup_cutter','place_cutter','pickup_server','place_server' | std_msgs/String.msg |
| /operation_status | Status of the current operation (success/executing/failure) | TBD |
