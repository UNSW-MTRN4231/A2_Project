#include <memory>
#include <chrono>
#include <thread>
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

auto generatePoseMsg(float x,float y, float z,float qx,float qy,float qz,float qw) {
    geometry_msgs::msg::Pose msg;
    msg.orientation.x = qx;
    msg.orientation.y = qy;
    msg.orientation.z = qz;
    msg.orientation.w = qw;
    msg.position.x = x;
    msg.position.y = y;
    msg.position.z = z;
    return msg;
}

void wait(int t) {
  std::chrono::seconds duration(t);
  std::this_thread::sleep_for(duration);
  return;
}

// Rotates a point theta (degrees) around a centre, in the XY plane
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

// Manual implementation of tf2::convert from geometry_msgs::msg::Quaternion to tf2::Quaternion
void convertGeometryMsgsToTF2(const geometry_msgs::msg::Quaternion& geometry_quat, tf2::Quaternion& tf_quat) {
    tf_quat.setX(geometry_quat.x);
    tf_quat.setY(geometry_quat.y);
    tf_quat.setZ(geometry_quat.z);
    tf_quat.setW(geometry_quat.w);
}

// Manual implementation of tf2::convert from tf2::Quaternion to geometry_msgs::msg::Quaternion
void convertTF2ToGeometryMsgs(const tf2::Quaternion& tf_quat, geometry_msgs::msg::Quaternion& geometry_quat) {
    geometry_quat.x = tf_quat.x();
    geometry_quat.y = tf_quat.y();
    geometry_quat.z = tf_quat.z();
    geometry_quat.w = tf_quat.w();
}

geometry_msgs::msg::Quaternion combineQuaternions(const geometry_msgs::msg::Quaternion& quaternion1, const geometry_msgs::msg::Quaternion& quaternion2) {
    tf2::Quaternion tf_quat1, tf_quat2;
    convertGeometryMsgsToTF2(quaternion1, tf_quat1);
    convertGeometryMsgsToTF2(quaternion2, tf_quat2);

    tf2::Quaternion combined_quat = tf_quat1 * tf_quat2;

    geometry_msgs::msg::Quaternion combined_msg;
    convertTF2ToGeometryMsgs(combined_quat,combined_msg);
    

    return combined_msg;
}

geometry_msgs::msg::Quaternion RPYToQuaternion(double roll, double pitch, double yaw) {
    tf2::Quaternion tf_quat;
    tf_quat.setRPY(roll, pitch, yaw);

    geometry_msgs::msg::Quaternion quat_msg;
    convertTF2ToGeometryMsgs(tf_quat,quat_msg);

    return quat_msg;
}

geometry_msgs::msg::Quaternion get_cut_quaternion(float yaw_angle) {
  geometry_msgs::msg::Quaternion down_rotation = RPYToQuaternion(0,M_PI,0);
  geometry_msgs::msg::Quaternion yaw_rotation = RPYToQuaternion(0,0,yaw_angle);

  geometry_msgs::msg::Quaternion combined = combineQuaternions(yaw_rotation,down_rotation);
  return combined;
}

geometry_msgs::msg::Quaternion get_serve_pick_quaternion(float yaw_angle) {
  float inclination_angle = (2.0/3.0) * M_PI;
  geometry_msgs::msg::Quaternion inclination_rotation = RPYToQuaternion(0,inclination_angle,0);
  geometry_msgs::msg::Quaternion yaw_rotation = RPYToQuaternion(0,0,yaw_angle);

  geometry_msgs::msg::Quaternion combined = combineQuaternions(yaw_rotation,inclination_rotation);
  return combined;
}

/////////////////////////////////////////////////////////////////////
//                  MOVEIT_TRAJECTORY CLASS MEMBERS                //
/////////////////////////////////////////////////////////////////////

moveit_trajectory::moveit_trajectory() : Node("moveit_trajectory") {

  // Initalise the transformation listener
  tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
  tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_);

  // Look up the transformation ever 200 milliseconds
  // timer_ = this->create_wall_timer( std::chrono::milliseconds(200), std::bind(&moveit_trajectory::tfCallback, this));
  operation_command_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "operation_command", 10, std::bind(&moveit_trajectory::operation_command_callback, this, std::placeholders::_1));
  operation_status_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "operation_status", 10, std::bind(&moveit_trajectory::operation_status_callback, this, std::placeholders::_1));
  pizza_radius_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "pizza_radius", 10, std::bind(&moveit_trajectory::set_pizza_radius, this, std::placeholders::_1));
  pizza_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "pizza_pose", 10, std::bind(&moveit_trajectory::set_pizza_pose, this, std::placeholders::_1));

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

// Plans and executes end effector motion from current pose to target pose, without restriction
void moveit_trajectory::move_to_pose(geometry_msgs::msg::Pose target_pose) {
  auto success = false;

  moveit::planning_interface::MoveGroupInterface::Plan planMessage;

  // visualize current and target pose
  auto current_pose = move_group_interface->getCurrentPose();
  //geometry_msgs::msg::Pose current_pose = get_end_effector_pose();
  //std::cout<<"Pose: "<<current_pose.position.x<<'\t'<<current_pose.position.y<<'\t'<<current_pose.position.z<<'\t'<<std::endl;
  visual_tools_->publishSphere(current_pose.pose, rviz_visual_tools::BLUE, 0.05);
  visual_tools_->publishSphere(target_pose, rviz_visual_tools::RED, 0.05);
  visual_tools_->trigger();

  // Plan movement to point
  move_group_interface->setPoseTarget(target_pose);
  success = static_cast<bool>(move_group_interface->plan(planMessage));

  // Execute movement to point
  if (success) {
    move_group_interface->execute(planMessage);
  } else {
    RCLCPP_ERROR(this->get_logger(), "Planning failed!");
  }

  RCLCPP_INFO(this->get_logger(), "Finished moving to point");

  // Delete all markers
  visual_tools_->deleteAllMarkers();
  visual_tools_->trigger(); 
}

// Plans and executes end effector motion from current pose to target pose. End effector is constrained
// to a cubic volume
void moveit_trajectory::move_to_pose_box_constraint(geometry_msgs::msg::Pose target_pose) {
    
    // Create box constraint
    moveit_msgs::msg::PositionConstraint box_constraint;
    box_constraint.header.frame_id = move_group_interface->getPoseReferenceFrame();
    box_constraint.link_name = move_group_interface->getEndEffectorLink();
    shape_msgs::msg::SolidPrimitive box;
    box.type = shape_msgs::msg::SolidPrimitive::BOX;
    box.dimensions = { 0.4, 0.8, 0.8 };
    box_constraint.constraint_region.primitives.emplace_back(box);

    // Define box pose
    geometry_msgs::msg::Pose box_pose;
    box_pose.position.x = target_pose.position.x;
    box_pose.position.y = 0.4;
    box_pose.position.z = 0.4;
    box_constraint.constraint_region.primitive_poses.emplace_back(box_pose);
    box_constraint.weight = 1.0; // Weird this doesnt effect anything 

    // Visualize box constraint
    Eigen::Vector3d box_point_1(box_pose.position.x - box.dimensions[0] / 2, box_pose.position.y - box.dimensions[1] / 2,
                                box_pose.position.z - box.dimensions[2] / 2);
    Eigen::Vector3d box_point_2(box_pose.position.x + box.dimensions[0] / 2, box_pose.position.y + box.dimensions[1] / 2,
                                box_pose.position.z + box.dimensions[2] / 2);
    visual_tools_->publishCuboid(box_point_1, box_point_2, rviz_visual_tools::TRANSLUCENT_DARK);
    visual_tools_->trigger();

    // Add constraint to planning scene
    moveit_msgs::msg::Constraints box_constraints;
    box_constraints.position_constraints.emplace_back(box_constraint);
    move_group_interface->setPathConstraints(box_constraints);

    // Plan and execute motion
    move_to_pose(target_pose);

    // Remove constraint for subsequent movements
    move_group_interface->clearPathConstraints();
}

void moveit_trajectory::move_to_pose_cartesian(std::vector<geometry_msgs::msg::Pose> waypoints, std::string ns) {

  // Plan the trajectory
  moveit_msgs::msg::RobotTrajectory trajectory;
  const double jump_threshold = 0.0; // Disabled jump threshold
  const double eef_step = 0.01; // Resolution the path will be interpolated at
  move_group_interface->computeCartesianPath(waypoints, eef_step, jump_threshold, trajectory);

  // Visualize the plan in RViz
  visualize_cartesian_path(waypoints,ns);

  // Execute the trajectory
  move_group_interface->execute(trajectory);

  // Note that this function will return *before* the trajectory is completely executed.
  // Planning new trajectories before the current one is completely executed leads to errors.
  // Therefore, include some delay after calling this function to ensure trajectory is completed
  // before program progresses.
}

void moveit_trajectory::set_orientation(geometry_msgs::msg::Quaternion target_orientation) {
  // Not implemented
  return;
}

void moveit_trajectory::setOrientationConstraint(std::string desired_orientation) {
  moveit_msgs::msg::OrientationConstraint ocm;
  ocm.link_name = "tool0";
  ocm.header.frame_id = "world";

  if (desired_orientation == "down") {
    ocm.orientation.x = 0;
    ocm.orientation.y = 1;
    ocm.orientation.z = 0;
    ocm.orientation.w = 0;
  } else {
    RCLCPP_ERROR(this->get_logger(),"Desired orientation for constraint recognised");
  }
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = M_PI;
  ocm.weight = 1.0;

  // Set as a path constraint for the group
  moveit_msgs::msg::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  move_group_interface->setPathConstraints(test_constraints);
}

geometry_msgs::msg::Pose moveit_trajectory::get_end_effector_pose(){
  std::string fromFrameRel = "world";
  std::string toFrameRel = "tool0";

  geometry_msgs::msg::TransformStamped t;

  try {
    t = tf_buffer_->lookupTransform( toFrameRel, fromFrameRel, tf2::TimePointZero);
  } catch (const tf2::TransformException & ex) {
    RCLCPP_INFO( this->get_logger(), "Could not transform %s to %s: %s", toFrameRel.c_str(), fromFrameRel.c_str(), ex.what());
  }

  geometry_msgs::msg::Pose p;
  p.position.x = t.transform.translation.x;
  p.position.y = -t.transform.translation.y; // No idea why this is negative, but it's working!
  p.position.z = t.transform.translation.z;
  p.orientation.x = t.transform.rotation.x;
  p.orientation.y = t.transform.rotation.y;
  p.orientation.z = t.transform.rotation.z;
  p.orientation.w = t.transform.rotation.w;
  return p;
}

/////////////////////////////////////////////////////////////////////
//                       TRAJECTORY PLANNING                       //
/////////////////////////////////////////////////////////////////////

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
    geometry_msgs::msg::Quaternion pick_orientation = get_serve_pick_quaternion(pick_angle);
    serve_pick_orientations.push_back(pick_orientation);
  }

  serve_picking_is_planned = true;
  return;
}

/////////////////////////////////////////////////////////////////////
//                       TRAJECTORY EXECUTION                      //
/////////////////////////////////////////////////////////////////////

void moveit_trajectory::cut_pizza() {
  float centre_pose_height = 0.2;
  float lift_height  = 0.04; // Height of vertical lead in and lead out

  // Create 'down' orientation
  geometry_msgs::msg::Quaternion down_orientation;
  down_orientation.x = 0;
  down_orientation.y = 1;
  down_orientation.z = 0;
  down_orientation.w = 0;

  // Create centre pose (returned to between cuts)
  geometry_msgs::msg::Pose centre_pose = pizza_pose;
  centre_pose.position.z = pizza_pose.position.z + centre_pose_height;;
  centre_pose.orientation = down_orientation;

  // Loop through cut points and execute
  for (size_t i = 0; i<cut_points.size(); i++) {
    std::cout<<"Executing cut "<< i << std::endl;

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
    move_to_pose_cartesian(waypoints, "Cutting Path");

    wait(5);
  }

  return;
}

void moveit_trajectory::pick_slice() {
  float centre_pose_height = 0.2;
  float lift_height  = 0.08; // Height of vertical lead in and lead out

  // Create centre pose, with orientation for spatula
  geometry_msgs::msg::Pose centre_pose = pizza_pose;
  centre_pose.position.z = pizza_pose.position.z + centre_pose_height;
  centre_pose.orientation = serve_pick_orientations.at(0);

  // Plan trajectory for the first set of pick points
  std::cout<<"Executing pick"<<std::endl;

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
  waypoints.push_back(centre_pose);
  waypoints.push_back(above_pick_start);
  waypoints.push_back(pick_start);
  waypoints.push_back(pick_end);
  waypoints.push_back(above_pick_end);
  waypoints.push_back(centre_pose);
  move_to_pose_cartesian(waypoints, "Pick path");

  wait(5);

  // Remove this slice from stored trajectories
  if (!serve_pick_points.empty()) {
    serve_pick_points.erase(serve_pick_points.begin());
  }
  if (!serve_pick_orientations.empty()) {
    serve_pick_orientations.erase(serve_pick_orientations.begin());
  }

  return;
}

/////////////////////////////////////////////////////////////////////
//                          VISUALIZATION                          //
/////////////////////////////////////////////////////////////////////

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

void moveit_trajectory::visualize_cartesian_path(std::vector<geometry_msgs::msg::Pose> waypoints,std::string ns){
  // Publish lines
  visual_tools_->publishPath(waypoints, rviz_visual_tools::LIME_GREEN, rviz_visual_tools::SMALL,ns);

  // Label points
  bool show_axes = false;
  if (show_axes) {
    for (size_t i = 0; i < waypoints.size(); i++) {
       visual_tools_->publishAxis(waypoints[i], rviz_visual_tools::SMALL, ns);
    }
  }
  visual_tools_->trigger();
}

void moveit_trajectory::visualize_pizza() {
  RCLCPP_INFO(this->get_logger(), "Vizualizing Pizza");

  // Pizza
  visual_tools_->publishCylinder(
    pizza_pose, rviz_visual_tools::RED,0.01,pizza_radius.data*2,"Pizza");

  // Centre
  visual_tools_->publishSphere(pizza_pose,rviz_visual_tools::GREEN,0.02,"Pizza Centre");

  visual_tools_->trigger();
}

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
void moveit_trajectory::set_pizza_radius(std_msgs::msg::Float32 pizza_radius){
  RCLCPP_INFO(this->get_logger(), "Pizza radius set");
  this->pizza_radius = pizza_radius;
}

// Callback for /pizza_pose
void moveit_trajectory::set_pizza_pose(geometry_msgs::msg::Pose pizza_pose){
  RCLCPP_INFO(this->get_logger(), "Pizza pose set");
  this->pizza_pose = pizza_pose;
}

// Callback for /operation_command topic
void moveit_trajectory::operation_command_callback(std_msgs::msg::String operation_command)
{
  if (operation_command.data == "Cut") {
    RCLCPP_INFO(this->get_logger(), "Executing Cuts");
    draw_title("Cutting");
    cut_pizza();
  }

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

    if (!cutting_is_planned){
      // Plan cutting
      RCLCPP_INFO(this->get_logger(), "Planning Cuts");
      draw_title("Planning_Cuts");
      plan_cuts();

      // Visualize
      RCLCPP_INFO(this->get_logger(), "Visualising Cuts");
      visualize_cut_points();
    }

    if (!serve_picking_is_planned){
      // Plan serve picking
      RCLCPP_INFO(this->get_logger(), "Planning Serve Pick");
      draw_title("Planning_Serve_Pick");
      plan_serve_pick();

      RCLCPP_INFO(this->get_logger(), "Visualising Serve Pick");
      visualize_serve_pick_points();
    }
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