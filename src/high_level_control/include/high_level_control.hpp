#ifndef HIGH_LEVEL_CONTROL_HPP
#define HIGH_LEVEL_CONTROL_HPP

#include <vector>
#include <mutex>
#include <string>

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

    void generate_operation_sequence();
    void update_detection_status();
    void execute_next_operation();

    // Subscription callbacks
    void pizza_radius_callback(std_msgs::msg::Float32 radius);
    void pizza_pose_callback(geometry_msgs::msg::Pose pose);
    void tool_jig_pose_callback(geometry_msgs::msg::Pose pose);
    void operation_status_callback(std_msgs::msg::String status);
    void keyboard_input_callback(std_msgs::msg::String key);

    /////////////////////////////////////////////////////////////////////
    //                            VARIABLES                            //
    /////////////////////////////////////////////////////////////////////

    // Mutex
    std::mutex operation_sequence_mutex;

    // Callback groups
    rclcpp::CallbackGroup::SharedPtr keyboard_cb_group;
    rclcpp::CallbackGroup::SharedPtr status_cb_group;

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_command_publisher_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_status_publisher_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr pizza_radius_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pizza_pose_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr tool_jig_pose_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_input_subscription_;

    // State checking
    bool pizza_radius_is_set = false;
    bool pizza_pose_is_set = false;
    bool tool_jig_pose_is_set = false;
    bool detection_is_complete = false;

    // Operation Sequence
    std::vector<std::string> operation_sequence;

};

#endif // HIGH_LEVEL_CONTROL_HPP
