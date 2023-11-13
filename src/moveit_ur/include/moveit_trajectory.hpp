#ifndef MOVEIT_TRAJECTORY_HPP
#define MOVEIT_TRAJECTORY_HPP

#include <string>
#include <vector>
#include <thread>
#include <mutex>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <sensor_msgs/msg/joint_state.hpp>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class moveit_trajectory : public rclcpp::Node
{
  public:
    // Constructor
    moveit_trajectory();

  private:
    /////////////////////////////////////////////////////////////////////
    //                             METHODS                             //
    /////////////////////////////////////////////////////////////////////

    // Basic movement
    void follow_path_cartesian(std::vector<geometry_msgs::msg::Pose> waypoints, std::string ns);
    void rotate_joint(std::string joint, float theta);
    void invert_wrist();
    void uninvert_wrist();

    // Trajectory planning
    geometry_msgs::msg::Quaternion get_cut_quaternion(float yaw);
    geometry_msgs::msg::Quaternion get_serve_quaternion(float inclination, float yaw);
    void plan_cuts();
    void plan_serve();

    // Trajectory execution
    void cut_pizza();
    void pick_slice();
    void place_slice();
    void pick_cutting_tool();
    void place_cutting_tool();
    void pick_serving_tool();
    void place_serving_tool();

    // Visualization
    void draw_title(std::string text);
    void visualize_cartesian_path(std::vector<geometry_msgs::msg::Pose> waypoints, std::string ns);

    void visualize_pizza();
    void visualize_plate();
    void visualize_cut_points();
    void visualize_serve_pick_points();

    // Subscription callbacks
    void joint_states_callback(sensor_msgs::msg::JointState joint_state_msg);
    void operation_command_callback(std_msgs::msg::String operation_command);
    void operation_status_callback(std_msgs::msg::String operation_status);
    void pizza_radius_callback(std_msgs::msg::Float32 pizza_radius);
    void pizza_pose_callback(geometry_msgs::msg::Pose pizza_pose);
    void tool_jig_pose_callback(geometry_msgs::msg::Pose tool_jig_pose);
    void plate_pose_callback(geometry_msgs::msg::Pose plate_pose);

    /////////////////////////////////////////////////////////////////////
    //                            VARIABLES                            //
    /////////////////////////////////////////////////////////////////////

    /////////////////////////////////////////////////////////////////////

    // Modifiable Trajectory Parameters (WIP - TODO collect all parameters here for easy modification)
    // General
    float flat_spatula_angle = (2.0/3.0) * M_PI; // Pitch (CCW from vertical) of the end effector for the spatula to be flat

    // Serving slice on plate
    float serve_height = 0.03;  // Height of spatula tip above the plate
    float serve_incline = M_PI/4; // Inclination of the spatula (below horizontal) when 'tipping' a slice onto the plate
    float serve_retreat = 0.1;  // Distance to 'pull out' spatula after tipping

    /////////////////////////////////////////////////////////////////////

    // Reading robot state from moveit
    std::mutex mutex_;

    // Moveit
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr state_cb_group;
    rclcpp::CallbackGroup::SharedPtr operation_cb_group;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_status_publisher_;

    // Subscriptions
    rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr joint_states_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pizza_radius_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pizza_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr plate_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tool_jig_pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_command_subscription_;

    // Stored variables from messages
    std_msgs::msg::Float32 pizza_radius;
    geometry_msgs::msg::Pose pizza_pose;
    geometry_msgs::msg::Pose plate_pose;
    geometry_msgs::msg::Pose tool_jig_pose;
    std::vector<double> joint_positions; // Order - shoulder lift, elbow, wrist 1, wrist 2, wrist 3, shoulder pan

    // Completion checking
    bool cutting_is_planned = false;
    bool serve_is_planned = false;

    // State checking
    bool wrist_is_inverted = false;

    // Trajectory planning
    // General
    int num_slices = 8;
    geometry_msgs::msg::Pose centre_pose;
    // Tool change
    geometry_msgs::msg::Pose cutting_tool_pose;
    geometry_msgs::msg::Pose serving_tool_pose;
    // Cutting
    // 2D vector of points, representing the start and end of each cut
    std::vector<std::vector<geometry_msgs::msg::Point>> cut_points;
    std::vector<geometry_msgs::msg::Quaternion> cut_orientations;
    // Serving (pick)
    // 2D vector of points, representing the start and end of each motion when picking up each slice
    std::vector<std::vector<geometry_msgs::msg::Point>> serve_pick_points;
    std::vector<geometry_msgs::msg::Quaternion> serve_pick_orientations;
    std::vector<bool> serve_invert_flags;
    // Serving (place)
    float plate_radius = 0.1; // TODO measure this
    std::vector<geometry_msgs::msg::Quaternion> serve_place_orientations;
    std::vector<bool> should_invert_place_direction;
};

#endif // MOVEIT_TRAJECTORY_HPP
