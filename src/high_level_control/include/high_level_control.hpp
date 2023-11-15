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
    void execute_next_operation();

    // Subscription callbacks
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
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr gui_text_publisher_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_status_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr keyboard_input_subscription_;

    // State checking
    bool previous_operation_complete = true;

    // Operation Sequence
    int num_slices = 8;
    std::vector<std::string> operation_sequence;

};

#endif // HIGH_LEVEL_CONTROL_HPP
