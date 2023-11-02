#ifndef MOVEIT_TRAJECTORY_HPP
#define MOVEIT_TRAJECTORY_HPP

#include <string>
#include <vector>

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

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

    // Trajectory planning
    geometry_msgs::msg::Quaternion get_cut_quaternion(float yaw);
    geometry_msgs::msg::Quaternion get_serve_pick_quaternion(float yaw);
    void plan_cuts();
    void plan_serve_pick();

    // Trajectory execution
    void cut_pizza();
    void pick_slice();

    // Visualization
    void draw_title(std::string text);
    void visualize_cartesian_path(std::vector<geometry_msgs::msg::Pose> waypoints, std::string ns);

    void visualize_pizza();
    void visualize_cut_points();
    void visualize_serve_pick_points();

    // Subscription callbacks
    void operation_command_callback(std_msgs::msg::String operation_command);
    void operation_status_callback(std_msgs::msg::String operation_status);
    void set_pizza_radius(std_msgs::msg::Float32 pizza_radius);
    void set_pizza_pose(geometry_msgs::msg::Pose pizza_pose);

    /////////////////////////////////////////////////////////////////////
    //                            VARIABLES                            //
    /////////////////////////////////////////////////////////////////////

    // Moveit
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;

    // Subscriptions and Buffers
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_command_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pizza_radius_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pizza_pose_subscription_;

    // Stored variables from messages
    std_msgs::msg::Float32 pizza_radius;
    geometry_msgs::msg::Pose pizza_pose;

    // State checking
    bool cutting_is_planned = false;
    bool serve_picking_is_planned = false;

    // Trajectory planning
    int num_slices = 8;
    // 2D vector of points, representing the start and end of each cut
    std::vector<std::vector<geometry_msgs::msg::Point>> cut_points;
    std::vector<geometry_msgs::msg::Quaternion> cut_orientations;
    // 2D vector of points, representing the start and end of each motion when picking up each slice
    std::vector<std::vector<geometry_msgs::msg::Point>> serve_pick_points;
    std::vector<geometry_msgs::msg::Quaternion> serve_pick_orientations;
};

#endif // MOVEIT_TRAJECTORY_HPP
