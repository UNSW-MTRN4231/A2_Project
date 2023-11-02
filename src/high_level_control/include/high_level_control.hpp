#ifndef HIGH_LEVEL_CONTROL_HPP
#define HIGH_LEVEL_CONTROL_HPP

#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class high_level_control : public rclcpp::Node
{
  public:
    // Constructor
    high_level_control();

  private:
    /////////////////////////////////////////////////////////////////////
    //                             METHODS                             //
    /////////////////////////////////////////////////////////////////////

    // Subscription callbacks
    void operation_status_callback(std_msgs::msg::String status);

    /////////////////////////////////////////////////////////////////////
    //                            VARIABLES                            //
    /////////////////////////////////////////////////////////////////////

    // Publishers
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr operation_command_publisher_;

    // Subscriptions
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_status_subscription_;

    // State checking
    bool detection_is_complete = false;

};

#endif // HIGH_LEVEL_CONTROL_HPP
