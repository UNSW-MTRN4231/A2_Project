#include <memory>
#include <chrono>
#include <vector>
#include <cmath>

#include "moveit_trajectory.hpp"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <geometry_msgs/msg/vector3.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <moveit_msgs/msg/display_robot_state.hpp>

using rclcpp::executors::MultiThreadedExecutor;

/////////////////////////////////////////////////////////////////////
//                              HELPERS                            //
/////////////////////////////////////////////////////////////////////

// Creates a collision object
auto generateCollisionObject(float sx,float sy, float sz, float x, float y, float z, std::string frame_id, std::string id) {
  moveit_msgs::msg::CollisionObject collision_object;
  collision_object.header.frame_id = frame_id;
  collision_object.id = id;
  shape_msgs::msg::SolidPrimitive primitive;

  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[primitive.BOX_X] = sx;
  primitive.dimensions[primitive.BOX_Y] = sy;
  primitive.dimensions[primitive.BOX_Z] = sz;

  geometry_msgs::msg::Pose box_pose;
  box_pose.orientation.w = 1.0; 
  box_pose.position.x = x;
  box_pose.position.y = y;
  box_pose.position.z = z;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  collision_object.operation = collision_object.ADD;

  return collision_object;
}

// Sleeps thread for a given number of seconds
void wait(int t) {
  std::chrono::seconds duration(t);
  std::this_thread::sleep_for(duration);
  return;
}

// Rotates a point an angle of theta (radians) around a centre point in the XY plane
geometry_msgs::msg::Point rotate_point(
  geometry_msgs::msg::Point point, geometry_msgs::msg::Point centre, float theta){

  // Translate to origin
  float tempX = point.x - centre.x;
  float tempY = point.y - centre.y;

  // Apply rotation
  float rotatedX = tempX * cos(theta) - tempY * sin(theta);
  float rotatedY = tempX * sin(theta) + tempY * cos(theta);

  // Translate back to the center
  geometry_msgs::msg::Point rotated_point;
  rotated_point.x = rotatedX + centre.x;
  rotated_point.y = rotatedY + centre.y;
  rotated_point.z = point.z; // Assuming z-coordinate remains the same

  return rotated_point;
}

// Conversion from geometry_msgs::msg::Quaternion to tf2::Quaternion
void convertGeometryMsgsToTF2(const geometry_msgs::msg::Quaternion& geometry_quat, tf2::Quaternion& tf_quat) {
    tf_quat.setX(geometry_quat.x);
    tf_quat.setY(geometry_quat.y);
    tf_quat.setZ(geometry_quat.z);
    tf_quat.setW(geometry_quat.w);
}

// Conversion from tf2::Quaternion to geometry_msgs::msg::Quaternion
void convertTF2ToGeometryMsgs(const tf2::Quaternion& tf_quat, geometry_msgs::msg::Quaternion& geometry_quat) {
    geometry_quat.x = tf_quat.x();
    geometry_quat.y = tf_quat.y();
    geometry_quat.z = tf_quat.z();
    geometry_quat.w = tf_quat.w();
}

// Combines 2 quaternions into a single quaternion
geometry_msgs::msg::Quaternion combineQuaternions(const geometry_msgs::msg::Quaternion& quaternion1, const geometry_msgs::msg::Quaternion& quaternion2) {
    tf2::Quaternion tf_quat1, tf_quat2;
    convertGeometryMsgsToTF2(quaternion1, tf_quat1);
    convertGeometryMsgsToTF2(quaternion2, tf_quat2);

    tf2::Quaternion combined_quat = tf_quat1 * tf_quat2;

    geometry_msgs::msg::Quaternion combined_msg;
    convertTF2ToGeometryMsgs(combined_quat,combined_msg);
    
    return combined_msg;
}

// Converts roll, pitch, yaw angles to a quaternion
geometry_msgs::msg::Quaternion RPYToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion quat_msg;
    convertTF2ToGeometryMsgs(tf_quat,quat_msg);

    return quat_msg;
}

geometry_msgs::msg::Pose add_translation_offset(geometry_msgs::msg::Pose pose, float x, float y, float z) {
  pose.position.x += x;
  pose.position.y += y;
  pose.position.z += z;

  return pose;
}

/////////////////////////////////////////////////////////////////////
//                  MOVEIT_TRAJECTORY CLASS MEMBERS                //
/////////////////////////////////////////////////////////////////////

moveit_trajectory::moveit_trajectory() : Node("moveit_trajectory") {
  // Callback groups
  state_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
  operation_cb_group = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);

  // Add callbacks to subscription options
  auto sub_options_state = rclcpp::SubscriptionOptions();
  sub_options_state.callback_group = state_cb_group;
  auto sub_options_operation = rclcpp::SubscriptionOptions();
  sub_options_operation.callback_group = operation_cb_group;

  // Publishers
  operation_status_publisher_ = this->create_publisher<std_msgs::msg::String>("operation_status", 10);
  arduino_command_publisher_ = this->create_publisher<std_msgs::msg::String>("arduino_command", 10);

  // Subscriptions
  joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&moveit_trajectory::joint_states_callback, this, std::placeholders::_1), sub_options_state);
  operation_command_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "operation_command", 10, std::bind(&moveit_trajectory::operation_command_callback, this, std::placeholders::_1),sub_options_operation);
  pizza_radius_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "pizza_radius", 10, std::bind(&moveit_trajectory::pizza_radius_callback, this, std::placeholders::_1),sub_options_operation);
  pizza_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "pizza_pose", 10, std::bind(&moveit_trajectory::pizza_pose_callback, this, std::placeholders::_1),sub_options_operation);
  plate_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "plate_pose", 10, std::bind(&moveit_trajectory::plate_pose_callback, this, std::placeholders::_1),sub_options_operation);
  tool_jig_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "tool_jig_pose", 10, std::bind(&moveit_trajectory::tool_jig_pose_callback, this, std::placeholders::_1),sub_options_operation);

  // Generate the movegroup interface
  move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::unique_ptr<rclcpp::Node>(this), "ur_manipulator");
  move_group_interface->setPlannerId("RRTConnectkConfigDefault");
  move_group_interface->setPlanningTime(10.0);
  move_group_interface->setNumPlanningAttempts(10);
  move_group_interface->setMaxVelocityScalingFactor(0.05);
  std::string frame_id = move_group_interface->getPlanningFrame();

  // Define collision objects (size, pos, frame, id)
  auto col_object_table = generateCollisionObject( 2.4, 1.2, 0.04, 0.85, 0.25, -0.03, frame_id, "table");
  auto col_object_backWall = generateCollisionObject( 2.4, 0.04, 1.0, 0.85, -0.45, 0.5, frame_id, "backWall");
  auto col_object_sideWall = generateCollisionObject( 0.04, 1.2, 1.0, -0.45, 0.25, 0.5, frame_id, "sideWall");

  // Add collision objects to planning scene
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  planning_scene_interface.applyCollisionObject(col_object_table);
  planning_scene_interface.applyCollisionObject(col_object_backWall);
  planning_scene_interface.applyCollisionObject(col_object_sideWall);

  // Instantiate visualization tool
  visual_tools_ = std::make_unique<moveit_visual_tools::MoveItVisualTools>(
    std::unique_ptr<rclcpp::Node>(this), 
    "base_link", 
    rviz_visual_tools::RVIZ_MARKER_TOPIC,
    move_group_interface->getRobotModel());
  visual_tools_->deleteAllMarkers();
  visual_tools_->loadRemoteControl();
}

/////////////////////////////////////////////////////////////////////
//                           BASIC MOVEMENT                        //
/////////////////////////////////////////////////////////////////////
void moveit_trajectory::follow_path_cartesian(std::vector<geometry_msgs::msg::Pose> waypoints, std::string ns) {

  // Plan the trajectory
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0; // Disabled jump threshold
  const double eef_step = 0.01; // Resolution the path will be interpolated at
  move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Visualize the plan in RViz
  visualize_cartesian_path(waypoints,ns);

  // Execute the trajectory
  move_group_interface->execute(trajectory);
}

void moveit_trajectory::rotate_joint(std::string joint, float theta) {
  std::vector<double> desired_joint_positions;

  // Get current joint positions
  // mutex lock
  std::lock_guard<std::mutex> lock(mutex_);
  desired_joint_positions.push_back(joint_positions[5]); // Shoulder pan
  desired_joint_positions.push_back(joint_positions[0]); // Shoulder lift
  desired_joint_positions.push_back(joint_positions[1]); // Elbow
  desired_joint_positions.push_back(joint_positions[2]); // Wrist 1
  desired_joint_positions.push_back(joint_positions[3]); // Wrist 2
  desired_joint_positions.push_back(joint_positions[4]); // Wrist 3
  // mutex unlock
  mutex_.unlock();

  // Calculate desired joint positions
  if (joint == "shoulder pan") {
    desired_joint_positions[0] += theta; 
  } else if (joint == "shoulder lift") {
    desired_joint_positions[1] += theta; 
  } else if (joint == "elbow") {
    desired_joint_positions[2] += theta; 
  } else if (joint == "wrist 1") {
    desired_joint_positions[3] += theta; 
  } else if (joint == "wrist 2") {
    desired_joint_positions[4] += theta; 
  } else if (joint == "wrist 3") {
    desired_joint_positions[5] += theta; 
  }
  // Check bounds
  bool within_bounds = move_group_interface->setJointValueTarget(desired_joint_positions);
  if (!within_bounds)
  {
    RCLCPP_WARN(this->get_logger(), "Target joint position(s) were outside of limits");
  }

  // Plan motion
  moveit::planning_interface::MoveGroupInterface::Plan joint_space_plan;
  auto success = (move_group_interface->plan(joint_space_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group_interface->move();
}


// Inverts wrist for greater clearance over table
void moveit_trajectory::invert_wrist(){

  rotate_joint("wrist 2", M_PI);
  wait(0.5);
  rotate_joint("wrist 1", M_PI/2);

  return;
}

// TODO need to place and pick tool
void moveit_trajectory::uninvert_wrist(){

  rotate_joint("wrist 1", -M_PI/2);
  wait(0.5);
  rotate_joint("wrist 2", -M_PI);

  return;
}


/////////////////////////////////////////////////////////////////////
//                       TRAJECTORY PLANNING                       //
/////////////////////////////////////////////////////////////////////

// Creates a quaternion which points down and with yaw angle given
geometry_msgs::msg::Quaternion moveit_trajectory::get_cut_quaternion(float yaw) {
  geometry_msgs::msg::Quaternion down_rotation = RPYToQuaternion(M_PI,0,0);
  geometry_msgs::msg::Quaternion yaw_rotation = RPYToQuaternion(0,0,yaw);

  geometry_msgs::msg::Quaternion combined = combineQuaternions(yaw_rotation,down_rotation);
  return combined;
}

// Creates a quaternion for which spatula is flat and with yaw angle given
geometry_msgs::msg::Quaternion moveit_trajectory::get_serve_quaternion(float inclination, float yaw) {
  geometry_msgs::msg::Quaternion inclination_rotation = RPYToQuaternion(0,inclination,0);
  geometry_msgs::msg::Quaternion yaw_rotation = RPYToQuaternion(0,0,yaw);

  geometry_msgs::msg::Quaternion combined = combineQuaternions(yaw_rotation,inclination_rotation);
  return combined;
}

// Plan slice centres, slicing operation start/end points on pizza, and slicing
// operation start/complete points (lead in and lead out)
void moveit_trajectory::plan_cuts() {

  int num_cuts = num_slices/2;
  float TCP_height = 0.12;

  // Define an initial, horizontal cut
  geometry_msgs::msg::Point start;
  start.x = pizza_pose.position.x - pizza_radius.data;
  start.y = pizza_pose.position.y;
  start.z = TCP_height;

  geometry_msgs::msg::Point cut_centre = pizza_pose.position;
  cut_centre.z = TCP_height;

  geometry_msgs::msg::Point end;
  end.x = pizza_pose.position.x + pizza_radius.data;
  end.y = pizza_pose.position.y;
  end.z = TCP_height;

  // Rotate the cut to find all cuts
  for (int i = 0; i<(num_cuts); i++){

    // Find start/end points
    float cut_angle = i * M_PI/num_cuts;
    geometry_msgs::msg::Point cut_start = rotate_point(start,pizza_pose.position,cut_angle);
    geometry_msgs::msg::Point cut_end = rotate_point(end,pizza_pose.position,cut_angle);

    // Store in vector
    std::vector<geometry_msgs::msg::Point> single_cut_points;
    single_cut_points.push_back(cut_start);
    single_cut_points.push_back(cut_centre);
    single_cut_points.push_back(cut_end);
    cut_points.push_back(single_cut_points);

    // Set orientation such that cutter blade is aligned with the cut direction
    geometry_msgs::msg::Quaternion cut_orientation = get_cut_quaternion(cut_angle);
    cut_orientations.push_back(cut_orientation);
  }

  cutting_is_planned = true;
}

void moveit_trajectory::plan_serve() {

  float TCP_height = 0.15; // Height of spatula tip
  float lead_in_length = 0.05; // Starting distance from spatula tip from pizza perimeter
  float spatula_length = 0.1; // Length of flat section of spatula, from the tip to the bend
  float slide_length = spatula_length + lead_in_length; // Total length of the sliding motion
  
  // Define an initial slice pickup
  geometry_msgs::msg::Point start;
  start.x = pizza_pose.position.x - pizza_radius.data - lead_in_length;
  start.y = pizza_pose.position.y;
  start.z = TCP_height;

  geometry_msgs::msg::Point end;
  end.x = start.x + slide_length;
  end.y = pizza_pose.position.y;
  end.z = TCP_height;

  // Rotate the cut to find all cuts
  for (int i = 0; i<(num_slices); i++){

    // Find start/end points
    float pick_angle = (i + 0.5) * (2*M_PI)/num_slices; // angle from which spatula approaches (in XY plane)
    geometry_msgs::msg::Point pick_start = rotate_point(start,pizza_pose.position,pick_angle);
    geometry_msgs::msg::Point pick_end = rotate_point(end,pizza_pose.position,pick_angle);

    // Store in vector
    std::vector<geometry_msgs::msg::Point> single_pick_points;
    single_pick_points.push_back(pick_start);
    single_pick_points.push_back(pick_end);
    serve_pick_points.push_back(single_pick_points);

    // Set orientation such that spatula is horizontal and points toward pizza centre
    geometry_msgs::msg::Quaternion pick_orientation = get_serve_quaternion(flat_spatula_angle,pick_angle);
    serve_pick_orientations.push_back(pick_orientation);

    // Determine if slice is on the half nearest the robot base
    bool in_half_near_robot_base = (pick_angle+M_PI/2-atan2(pizza_pose.position.y, pizza_pose.position.x)) < M_PI;
    // Set place orientation to avoid unplanned wrist inversions
    geometry_msgs::msg::Quaternion place_orientation;
    if (in_half_near_robot_base) {
      place_orientation = get_serve_quaternion(flat_spatula_angle,0);
      should_invert_place_direction.push_back(false);
    } else {
      place_orientation = get_serve_quaternion(flat_spatula_angle, M_PI);
      should_invert_place_direction.push_back(true);
    }
    serve_place_orientations.push_back(place_orientation);

    // Set 'invert wrist' flags
    bool should_invert_wrist = false;
    if (in_half_near_robot_base) {
      should_invert_wrist = true;
    }
    serve_invert_flags.push_back(should_invert_wrist);
  }

  serve_is_planned = true;
  return;
}

/////////////////////////////////////////////////////////////////////
//                       TRAJECTORY EXECUTION                      //
/////////////////////////////////////////////////////////////////////

// Sends string command to /arduino_command
void moveit_trajectory::send_gripper_command(std::string command) {
  std_msgs::msg::String msg;
  msg.data = command;
  arduino_command_publisher_->publish(msg);
}

// Completes all planned cuts of the pizza
void moveit_trajectory::cut_pizza() {
  float lift_height  = 0.04; // Height of vertical lead in and lead out

  // Loop through cut points and execute
  for (size_t i = 0; i<cut_points.size(); i++) {
    
    // Create waypoints
    geometry_msgs::msg::Pose above_cut_start;
    above_cut_start.position = cut_points.at(i).at(0);
    above_cut_start.position.z += lift_height;
    above_cut_start.orientation = cut_orientations.at(i);

    geometry_msgs::msg::Pose cut_start;
    cut_start.position = cut_points.at(i).at(0);
    cut_start.orientation = cut_orientations.at(i);

    geometry_msgs::msg::Pose cut_centre;
    cut_centre.position = cut_points.at(i).at(1);
    cut_centre.orientation = cut_orientations.at(i);

    geometry_msgs::msg::Pose cut_end;
    cut_end.position = cut_points.at(i).at(2);
    cut_end.orientation = cut_orientations.at(i);

    geometry_msgs::msg::Pose above_cut_end;
    above_cut_end.position = cut_points.at(i).at(2);
    above_cut_end.position.z += lift_height;
    above_cut_end.orientation = cut_orientations.at(i);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(centre_pose);
    waypoints.push_back(above_cut_start);
    waypoints.push_back(cut_start);
    waypoints.push_back(cut_centre);
    waypoints.push_back(cut_end);
    waypoints.push_back(above_cut_end);
    waypoints.push_back(centre_pose);

    // Execute path
    RCLCPP_INFO(this->get_logger(), "Executing cut");
    follow_path_cartesian(waypoints, "Cutting Path");

    std::cout<<"execute() returned"<<std::endl;
    wait(0.5);
  }

  return;
}

// Picks a single slice and removes it from the planned trajectories
void moveit_trajectory::pick_slice() {

  float lift_height  = 0.08; // Height of vertical lead in and lead out

  // Return if all slices picked up
  if (serve_pick_points.empty()) {
    RCLCPP_WARN(this->get_logger(), "No slice picking trajectories planned.");
    return;
  }
  
  // Set wrist inversion
  if (serve_invert_flags.at(0) && !wrist_is_inverted) {
    invert_wrist();
    wrist_is_inverted = true;
  } else if (!serve_invert_flags.at(0) && wrist_is_inverted) {
    uninvert_wrist();
    wrist_is_inverted = false;
  }

  // Create waypoints
  // Centre pose, with orientation for spatula
  geometry_msgs::msg::Pose centre_rotated_pose = centre_pose;
  centre_rotated_pose.orientation = serve_pick_orientations.at(0);

  geometry_msgs::msg::Pose above_pick_start;
  above_pick_start.position = serve_pick_points.at(0).at(0);
  above_pick_start.position.z += lift_height;
  above_pick_start.orientation = serve_pick_orientations.at(0);

  geometry_msgs::msg::Pose pick_start;
  pick_start.position = serve_pick_points.at(0).at(0);
  pick_start.orientation = serve_pick_orientations.at(0);

  geometry_msgs::msg::Pose pick_end;
  pick_end.position = serve_pick_points.at(0).at(1);
  pick_end.orientation = serve_pick_orientations.at(0);

  geometry_msgs::msg::Pose above_pick_end;
  above_pick_end.position = serve_pick_points.at(0).at(1);
  above_pick_end.position.z += lift_height;
  above_pick_end.orientation = serve_pick_orientations.at(0);

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(centre_rotated_pose);
  waypoints.push_back(above_pick_start);
  waypoints.push_back(pick_start);
  waypoints.push_back(pick_end);
  waypoints.push_back(above_pick_end);
  waypoints.push_back(centre_rotated_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Executing pick");
  follow_path_cartesian(waypoints, "Pick path");

  wait(0.5);

  // Remove picked slice from stored trajectories
  if (!serve_pick_points.empty()) {
    serve_pick_points.erase(serve_pick_points.begin());
  }
  if (!serve_pick_orientations.empty()) {
    serve_pick_orientations.erase(serve_pick_orientations.begin());
  }
  if (!serve_invert_flags.empty()) {
    serve_invert_flags.erase(serve_invert_flags.begin());
  }

  return;
}

void moveit_trajectory::place_slice() {
  // Create waypoints
  // Centre pose, with orientation for spatula
  geometry_msgs::msg::Pose centre_rotated_pose = centre_pose;
  centre_rotated_pose.orientation = serve_place_orientations.at(0);

  geometry_msgs::msg::Pose above_plate;
  above_plate.position = plate_pose.position;
  above_plate.position.z += serve_height;
  above_plate.orientation = serve_place_orientations.at(0);

  geometry_msgs::msg::Pose incline;
  incline.position = above_plate.position;
  if (!should_invert_place_direction.at(0)) {
    incline.orientation = get_serve_quaternion(flat_spatula_angle+serve_incline,0);
  } else {
    incline.orientation = get_serve_quaternion(flat_spatula_angle+serve_incline,M_PI);
  }
  

  geometry_msgs::msg::Pose slide_out;
  if (!should_invert_place_direction.at(0)) {
    slide_out.position.x = above_plate.position.x - (serve_retreat*sin(serve_incline));
  } else {
    slide_out.position.x = above_plate.position.x + (serve_retreat*sin(serve_incline));
  }
  slide_out.position.y = above_plate.position.y;
  slide_out.position.z = above_plate.position.z + (serve_retreat*cos(serve_incline));
  slide_out.orientation = incline.orientation;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(centre_rotated_pose);
  waypoints.push_back(above_plate);
  waypoints.push_back(incline);
  waypoints.push_back(slide_out);
  waypoints.push_back(centre_rotated_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Placing slice");
  follow_path_cartesian(waypoints, "Place path");

  wait(0.5);

  // Remove placed slice from stored trajectories
  if (!serve_place_orientations.empty()) {
    serve_place_orientations.erase(serve_place_orientations.begin());
  }
  if (!should_invert_place_direction.empty()) {
    should_invert_place_direction.erase(should_invert_place_direction.begin());
  }
  

  return;
}

void moveit_trajectory::pick_cutting_tool() {
  float approach_height = 0.15;
  
  // Create waypoints
  geometry_msgs::msg::Pose above_cutting_tool = cutting_tool_pose;
  above_cutting_tool.position.z += approach_height;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(centre_pose);
  waypoints.push_back(above_cutting_tool);
  waypoints.push_back(cutting_tool_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Approaching cutting tool");
  follow_path_cartesian(waypoints, "Cutting tool approach");

  // Close gripper
  send_gripper_command("close");
  wait(2); // TODO modify this duration

  waypoints.clear();
  waypoints.push_back(above_cutting_tool);
  waypoints.push_back(centre_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Returning to centre");
  follow_path_cartesian(waypoints, "Return to centre");
}

void moveit_trajectory::place_cutting_tool() {
  float approach_height = 0.15;
  
  // Create waypoints
  geometry_msgs::msg::Pose above_cutting_tool = cutting_tool_pose;
  above_cutting_tool.position.z += approach_height;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(centre_pose);
  waypoints.push_back(above_cutting_tool);
  waypoints.push_back(cutting_tool_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Approaching jig");
  follow_path_cartesian(waypoints, "Place cutting tool approach");

  // Open Gripper
  send_gripper_command("open");
  wait(2); // TODO modify this duration

  waypoints.clear();
  waypoints.push_back(above_cutting_tool);
  waypoints.push_back(centre_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Returning to centre");
  follow_path_cartesian(waypoints, "Return to centre");
}


void moveit_trajectory::pick_serving_tool() {
  float approach_height = 0.15;
  
  // Create waypoints
  geometry_msgs::msg::Pose above_serving_tool = serving_tool_pose;
  above_serving_tool.position.z += approach_height;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(centre_pose);
  waypoints.push_back(above_serving_tool);
  waypoints.push_back(serving_tool_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Approaching serving tool");
  follow_path_cartesian(waypoints, "Serving tool approach");

  // Close gripper
  send_gripper_command("close");
  wait(2); // TODO modify this duration

  waypoints.clear();
  waypoints.push_back(above_serving_tool);
  waypoints.push_back(centre_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Returning to centre");
  follow_path_cartesian(waypoints, "Return to centre");
}

void moveit_trajectory::place_serving_tool() {
  float approach_height = 0.15;
  
  // Create waypoints
  geometry_msgs::msg::Pose above_serving_tool = serving_tool_pose;
  above_serving_tool.position.z += approach_height;

  std::vector<geometry_msgs::msg::Pose> waypoints;
  waypoints.push_back(centre_pose);
  waypoints.push_back(above_serving_tool);
  waypoints.push_back(serving_tool_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Approaching tool jig");
  follow_path_cartesian(waypoints, "Place serving tool approach");

  // Open Gripper
  send_gripper_command("open");
  wait(2); // TODO modify this duration

  waypoints.clear();
  waypoints.push_back(above_serving_tool);
  waypoints.push_back(centre_pose);

  // Execute path
  RCLCPP_INFO(this->get_logger(), "Returning to centre");
  follow_path_cartesian(waypoints, "Return to centre");
}


/////////////////////////////////////////////////////////////////////
//                          VISUALIZATION                          //
/////////////////////////////////////////////////////////////////////

// Creates large text in RVIZ above the robot
void moveit_trajectory::draw_title(std::string text) {
  auto const text_pose = [] {
    auto msg = Eigen::Isometry3d::Identity();
    msg.translation().x() = 0.5;
    msg.translation().z() = 1.0;
    return msg;
  }();
  visual_tools_->publishText(text_pose, text, rviz_visual_tools::WHITE, rviz_visual_tools::XXLARGE);
  visual_tools_->trigger();
}

// Visualises a cartesian path in RVIZ with lines between each waypoint, and axes of each waypoint (optional)
void moveit_trajectory::visualize_cartesian_path(std::vector<geometry_msgs::msg::Pose> waypoints,std::string ns){
  // Publish lines
  visual_tools_->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL,ns);

  // Label points
  bool show_axes = true;
  if (show_axes) {
    for (size_t i = 0; i < waypoints.size(); i++) {
       visual_tools_->publishAxis(waypoints[i], rviz_visual_tools::SMALL, ns);
    }
  }
  visual_tools_->trigger();
}

// Visualises the pizza in RVIZ using a cylinder
void moveit_trajectory::visualize_pizza() {
  RCLCPP_INFO(this->get_logger(), "Vizualizing Pizza");

  // Pizza
  visual_tools_->publishCylinder(
    pizza_pose, rviz_visual_tools::RED,0.01,pizza_radius.data*2,"Pizza");

  // Centre
  visual_tools_->publishSphere(pizza_pose,rviz_visual_tools::GREEN,0.02,"Pizza Centre");

  visual_tools_->trigger();
}

// Visualises the plate in RVIZ using a cylinder
void moveit_trajectory::visualize_plate() {
  RCLCPP_INFO(this->get_logger(), "Vizualizing Plate");

  // Plate
  visual_tools_->publishCylinder(
    plate_pose, rviz_visual_tools::MAGENTA,0.01,plate_radius*2,"Plate");

  visual_tools_->trigger();
}

// Draws lines in RVIZ between the start and end points of each planned cut
void moveit_trajectory::visualize_cut_points() {
  RCLCPP_INFO(this->get_logger(), "Vizualizing Cut Points");

  for (size_t i = 0; i<cut_points.size(); i++) {

    // Get start/end points
    geometry_msgs::msg::Point cut_start = cut_points.at(i).at(0);
    geometry_msgs::msg::Point cut_end = cut_points.at(i).at(1);

    // Change z coordinate, so that the cut lines appear on top of the pizza cylinder
    cut_start.z += 0.012;
    cut_end.z += 0.012;
    visual_tools_->publishLine(cut_start,cut_end,rviz_visual_tools::BLUE,rviz_visual_tools::SMALL);
  }

  visual_tools_->trigger();
}

// Draws lines in RVIZ between the start and end points of the TCP for each planned serving pick
void moveit_trajectory::visualize_serve_pick_points() {
  RCLCPP_INFO(this->get_logger(), "Vizualizing Serve Pick Points");

  for (size_t i = 0; i<serve_pick_points.size(); i++) {

    // Get start/end points
    geometry_msgs::msg::Point pick_start = serve_pick_points.at(i).at(0);
    geometry_msgs::msg::Point pick_end = serve_pick_points.at(i).at(1);

    // Change z coordinate, so that the cut lines appear on top of the pizza cylinder
    pick_start.z += 0.012;
    pick_end.z += 0.012;
    visual_tools_->publishLine(pick_start,pick_end,rviz_visual_tools::CYAN,rviz_visual_tools::SMALL);
  }

  visual_tools_->trigger();
}

/////////////////////////////////////////////////////////////////////
//                      SUBSCRIPTION CALLBACKS                     //
/////////////////////////////////////////////////////////////////////

// Callback for /joint_states
void moveit_trajectory::joint_states_callback(sensor_msgs::msg::JointState joint_state_msg){
  std::lock_guard<std::mutex> lock(mutex_);
  
  joint_positions.clear();
  for (int i=0; i<6; i++) {
    joint_positions.push_back(joint_state_msg.position[i]);
  }
}

// Callback for /pizza_radius
void moveit_trajectory::pizza_radius_callback(std_msgs::msg::Float32 pizza_radius) {
  RCLCPP_INFO(this->get_logger(), "Pizza radius set");
  this->pizza_radius = pizza_radius;
}

// Callback for /pizza_pose
void moveit_trajectory::pizza_pose_callback(geometry_msgs::msg::Pose pizza_pose) {
  // Set pizza pose
  RCLCPP_INFO(this->get_logger(), "Pizza pose set");
  this->pizza_pose = pizza_pose;

  // Update centre pose (above pizza)
  centre_pose = pizza_pose;
  centre_pose.position.z += 0.3; // Clearance above pizza
  // Rotation
  geometry_msgs::msg::Quaternion down_orientation = RPYToQuaternion(0,M_PI,0);
  centre_pose.orientation = down_orientation;
}

// Callback for /plate_pose
void moveit_trajectory::plate_pose_callback(geometry_msgs::msg::Pose plate_pose) {
  RCLCPP_INFO(this->get_logger(), "Plate pose set");
  this->plate_pose = plate_pose;
}

// Callback for /tool_jig_pose
void moveit_trajectory::tool_jig_pose_callback(geometry_msgs::msg::Pose tool_jig_pose) {
  RCLCPP_INFO(this->get_logger(), "Tool jig pose set");
  this->tool_jig_pose = tool_jig_pose;

  this->cutting_tool_pose = add_translation_offset(tool_jig_pose,0,0,0); // TODO determine offset
  this->serving_tool_pose = add_translation_offset(tool_jig_pose,0,0,0); // TODO determine offset
}

// Callback for /operation_command topic
void moveit_trajectory::operation_command_callback(std_msgs::msg::String operation_command)
{
  // Planning
  if (operation_command.data == "Plan Trajectories") {
    draw_title("Planning_Trajectories");
    RCLCPP_INFO(this->get_logger(), "Planning Trajectories");

    // Visualize detections
    visualize_pizza();
    visualize_plate();
    // TODO visualize tool jig

    // Cut planning
    if (!cutting_is_planned){
      plan_cuts();
      visualize_cut_points();
    }

    // Serve picking planning
    if (!serve_is_planned){
      plan_serve();
      visualize_serve_pick_points();
    }

    // Publish to /operation_status
    std_msgs::msg::String msg;
    msg.data = "Plan Trajectories Complete";
    operation_status_publisher_->publish(msg);
  }

  // Pick cutting tool
  if (operation_command.data == "Pick Cutting Tool") {
    RCLCPP_INFO(this->get_logger(), "Picking Cutting Tool");
    draw_title("Pick_Cutting_Tool");
    pick_cutting_tool();

    // Publish to /operation_status
    std_msgs::msg::String msg;
    msg.data = "Pick Cutting Tool Complete";
    operation_status_publisher_->publish(msg);
  }

  // Cutting
  if (operation_command.data == "Cut") {
    RCLCPP_INFO(this->get_logger(), "Cutting");
    draw_title("Cutting");
    cut_pizza();

    // Publish to /operation_status
    std_msgs::msg::String msg;
    msg.data = "Cut Complete";
    operation_status_publisher_->publish(msg);
  }

  // Pick cutting tool
  if (operation_command.data == "Place Cutting Tool") {
    RCLCPP_INFO(this->get_logger(), "Placing Cutting Tool");
    draw_title("Place_Cutting_Tool");
    place_cutting_tool();

    // Publish to /operation_status
    std_msgs::msg::String msg;
    msg.data = "Place Cutting Tool Complete";
    operation_status_publisher_->publish(msg);
  }

  // Pick serving tool
  if (operation_command.data == "Pick Serving Tool") {
    RCLCPP_INFO(this->get_logger(), "Picking Serving Tool");
    draw_title("Pick_Serving_Tool");
    pick_serving_tool();

    // Publish to /operation_status
    std_msgs::msg::String msg;
    msg.data = "Pick Serving Tool Complete";
    operation_status_publisher_->publish(msg);
  }

  // Picking slice
  if (operation_command.data == "Pick Slice") {
    RCLCPP_INFO(this->get_logger(), "Picking Slice");
    draw_title("Picking_Slice");
    pick_slice();

    // Publish to /operation_status
    std_msgs::msg::String msg;
    msg.data = "Pick Slice Complete";
    operation_status_publisher_->publish(msg);
  }

  // Placing Slice
  if (operation_command.data == "Place Slice") {
    RCLCPP_INFO(this->get_logger(), "Placing Slice");
    draw_title("Placing_Slice");
    place_slice();

    // Publish to /operation_status
    std_msgs::msg::String msg;
    msg.data = "Place Slice Complete";
    operation_status_publisher_->publish(msg);
  }

  // Place serving tool
  if (operation_command.data == "Place Serving Tool") {
    RCLCPP_INFO(this->get_logger(), "Placinging Serving Tool");
    draw_title("Place_Serving_Tool");
    place_serving_tool();

    // Publish to /operation_status
    std_msgs::msg::String msg;
    msg.data = "Place Serving Tool Complete";
    operation_status_publisher_->publish(msg);
  }

  return;
}

/////////////////////////////////////////////////////////////////////
//                              MAIN                               //
/////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::executors::MultiThreadedExecutor executor;
  auto nh = std::make_shared<moveit_trajectory>();
  executor.add_node(nh);
  executor.spin();

  RCLCPP_INFO(nh->get_logger(),"Exiting node");
  rclcpp::shutdown();
  return 0;
}