#ifndef MOVEIT_TRAJECTORY_HPP
#define MOVEIT_TRAJECTORY_HPP

#include <string>

#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit_visual_tools/moveit_visual_tools.h>

#include "tf2/exceptions.h"
#include "tf2_ros/transform_listener.h"
#include "tf2_ros/buffer.h"

class moveit_trajectory : public rclcpp::Node
{
  public:
    moveit_trajectory();

  private:
    void move_to_pose(geometry_msgs::msg::Pose target_pose);
    void move_to_pose_box_constraint(geometry_msgs::msg::Pose target_pose);
    void move_to_pose_cartesian(geometry_msgs::msg::Pose target_pose);
    void set_orientation(geometry_msgs::msg::Quaternion target_orientation);
    void setOrientationConstraint(std::string desired_orientation);
    geometry_msgs::msg::Pose get_end_effector_pose();
    void move_callback(const geometry_msgs::msg::Pose::SharedPtr pose);

    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr subscription_;
    rclcpp::TimerBase::SharedPtr timer_;
    std::shared_ptr<tf2_ros::TransformListener> tf_listener_{nullptr};
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;
    std::unique_ptr<moveit::planning_interface::MoveGroupInterface> move_group_interface;
    std::unique_ptr<moveit_visual_tools::MoveItVisualTools> visual_tools_;
};

#endif // MOVEIT_TRAJECTORY_HPP
