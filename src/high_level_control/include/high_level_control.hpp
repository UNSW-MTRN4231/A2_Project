#ifndef HIGH_LEVEL_CONTROL_HPP
#define HIGH_LEVEL_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/pose.hpp>

class high_level_control : public rclcpp::Node
{
  public:
    // Constructor
    high_level_control();

  private:
    /////////////////////////////////////////////////////////////////////
    //                             METHODS                             //
    /////////////////////////////////////////////////////////////////////

    void update_detection_status();

    // Subscription callbacks
    void pizza_radius_callback(std_msgs::msg::Float32 radius);
    void pizza_pose_callback(geometry_msgs::msg::Pose pose);
    void operation_status_callback(std_msgs::msg::String status);

    /////////////////////////////////////////////////////////////////////
    //                            VARIABLES                            //
    /////////////////////////////////////////////////////////////////////

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_status_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_command_publisher_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pizza_radius_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pizza_pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_status_subscription_;

    // State checking
    bool pizza_radius_is_set = false;
    bool pizza_pose_is_set = false;
    bool detection_is_complete = false;

};

#endif // HIGH_LEVEL_CONTROL_HPP
