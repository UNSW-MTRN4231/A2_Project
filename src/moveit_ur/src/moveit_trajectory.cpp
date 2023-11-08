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

  // Subscriptions
  joint_states_subscription_ = this->create_subscription<sensor_msgs::msg::JointState>(
    "joint_states", 10, std::bind(&moveit_trajectory::joint_states_callback, this, std::placeholders::_1), sub_options_state);
  operation_command_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "operation_command", 10, std::bind(&moveit_trajectory::operation_command_callback, this, std::placeholders::_1),sub_options_operation);
  operation_status_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "operation_status", 10, std::bind(&moveit_trajectory::operation_status_callback, this, std::placeholders::_1),sub_options_operation);
  pizza_radius_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "pizza_radius", 10, std::bind(&moveit_trajectory::pizza_radius_callback, this, std::placeholders::_1),sub_options_operation);
  pizza_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "pizza_pose", 10, std::bind(&moveit_trajectory::pizza_pose_callback, this, std::placeholders::_1),sub_options_operation);

  // Generate the movegroup interface
  move_group_interface = std::make_unique<moveit::planning_interface::MoveGroupInterface>(std::unique_ptr<rclcpp::Node>(this), "ur_manipulator");
  move_group_interface->setPlannerId("RRTConnectkConfigDefault");
  move_group_interface->setPlanningTime(10.0);
  move_group_interface->setNumPlanningAttempts(10);
  move_group_interface->setMaxVelocityScalingFactor(0.1);
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
  desired_joint_positions = joint_positions;
  // mutex unlock
  mutex_.unlock();

  // Calculate desired joint positions
  if (joint == "shoulder lift") {
    desired_joint_positions[0] += theta; 
  } else if (joint == "elbow") {
    desired_joint_positions[1] += theta; 
  } else if (joint == "wrist 1") {
    desired_joint_positions[2] += theta; 
  } else if (joint == "wrist 2") {
    desired_joint_positions[3] += theta; 
  } else if (joint == "wrist 3") {
    desired_joint_positions[4] += theta; 
  } else if (joint == "shoulder pan") {
    desired_joint_positions[5] += theta; 
  }
  
  std::vector<double> test_only;
  test_only.push_back(0.0);
  test_only.push_back(-0.2);
  test_only.push_back(0.0);
  test_only.push_back(0.0);
  test_only.push_back(M_PI/2);
  test_only.push_back(0.0);

  // Check bounds
  bool within_bounds = move_group_interface->setJointValueTarget(test_only);
  if (!within_bounds)
  {
    RCLCPP_WARN(this->get_logger(), "Target joint position(s) were outside of limits, but we will plan and clamp to the limits ");
  }

  // Plan motion
  moveit::planning_interface::MoveGroupInterface::Plan joint_space_plan;
  auto success = (move_group_interface->plan(joint_space_plan) == moveit::core::MoveItErrorCode::SUCCESS);
  move_group_interface->move();
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
geometry_msgs::msg::Quaternion moveit_trajectory::get_serve_pick_quaternion(float inclination, float yaw) {
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
    single_cut_points.push_back(cut_end);
    cut_points.push_back(single_cut_points);

    // Set orientation such that cutter blade is aligned with the cut direction
    geometry_msgs::msg::Quaternion cut_orientation = get_cut_quaternion(cut_angle);
    cut_orientations.push_back(cut_orientation);
  }

  cutting_is_planned = true;
}

void moveit_trajectory::plan_serve_pick() {

  float TCP_height = 0.15; // Height of spatula tip
  float lead_in_length = 0.05; // Starting distance from spatula tip from pizza perimeter
  float spatula_length = 0.1; // Length of flat section of spatula, from the tip to the bend
  float slide_length = spatula_length + lead_in_length; // Total length of the sliding motion
  float inclination_angle = (2.0/3.0) * M_PI;
  
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
    geometry_msgs::msg::Quaternion pick_orientation = get_serve_pick_quaternion(inclination_angle,pick_angle);
    serve_pick_orientations.push_back(pick_orientation);
  }

  serve_picking_is_planned = true;
  return;
}

/////////////////////////////////////////////////////////////////////
//                       TRAJECTORY EXECUTION                      //
/////////////////////////////////////////////////////////////////////

// Inverts wrist for greater clearance over table
void moveit_trajectory::invert_wrist(){

  rotate_joint("shoulder pan", 0.1);

  return;
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

    geometry_msgs::msg::Pose cut_end;
    cut_end.position = cut_points.at(i).at(1);
    cut_end.orientation = cut_orientations.at(i);

    geometry_msgs::msg::Pose above_cut_end;
    above_cut_end.position = cut_points.at(i).at(1);
    above_cut_end.position.z += lift_height;
    above_cut_end.orientation = cut_orientations.at(i);

    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(centre_pose);
    waypoints.push_back(above_cut_start);
    waypoints.push_back(cut_start);
    waypoints.push_back(cut_end);
    waypoints.push_back(above_cut_end);
    waypoints.push_back(centre_pose);

    // Execute path
    RCLCPP_INFO(this->get_logger(), "Executing cut");
    follow_path_cartesian(waypoints, "Cutting Path");

    wait(5);
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

  // TODO remove FOR TESTING ONLY!!!
  invert_wrist();

  /*

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

  wait(5);

  // Remove picked slice from stored trajectories
  if (!serve_pick_points.empty()) {
    serve_pick_points.erase(serve_pick_points.begin());
  }
  if (!serve_pick_orientations.empty()) {
    serve_pick_orientations.erase(serve_pick_orientations.begin());
  }

  */

  return;
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

// Callback for /pizza_radius
void moveit_trajectory::pizza_radius_callback(std_msgs::msg::Float32 pizza_radius){
  RCLCPP_INFO(this->get_logger(), "Pizza radius set");
  this->pizza_radius = pizza_radius;
}

// Callback for /pizza_pose
void moveit_trajectory::pizza_pose_callback(geometry_msgs::msg::Pose pizza_pose){
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

// Callback for /operation_command topic
void moveit_trajectory::operation_command_callback(std_msgs::msg::String operation_command)
{
  // Cutting
  if (operation_command.data == "Cut") {
    RCLCPP_INFO(this->get_logger(), "Executing Cuts");
    draw_title("Cutting");
    cut_pizza();
  }

  // Picking slice
  if (operation_command.data == "Pick Slice") {
    RCLCPP_INFO(this->get_logger(), "Picking Slice");
    draw_title("Picking_Slice");
    pick_slice();
  }

  return;
}

// Callback for /operation_status topic
void moveit_trajectory::operation_status_callback(std_msgs::msg::String operation_status)
{ 
  if (operation_status.data == "Detection Complete") {
    draw_title("Detection_Complete");
    RCLCPP_INFO(this->get_logger(), "Detection Complete");
    visualize_pizza();

    // Cut planning
    if (!cutting_is_planned){
      RCLCPP_INFO(this->get_logger(), "Planning Cuts");
      draw_title("Planning_Cuts");
      plan_cuts();

      // Visualize
      RCLCPP_INFO(this->get_logger(), "Visualising Cuts");
      visualize_cut_points();
    }

    // Serve picking planning
    if (!serve_picking_is_planned){
      RCLCPP_INFO(this->get_logger(), "Planning Serve Pick");
      draw_title("Planning_Serve_Pick");
      plan_serve_pick();

      RCLCPP_INFO(this->get_logger(), "Visualising Serve Pick");
      visualize_serve_pick_points();
    }
  }

  return;
}

void moveit_trajectory::joint_states_callback(sensor_msgs::msg::JointState joint_state_msg){
  std::lock_guard<std::mutex> lock(mutex_);
  
  for (int i=0; i<6; i++) {
    joint_positions.push_back(joint_state_msg.position[i]);
  }
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