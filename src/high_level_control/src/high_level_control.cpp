#include <thread>

#include "high_level_control.hpp"

/////////////////////////////////////////////////////////////////////
//                              HELPERS                            //
/////////////////////////////////////////////////////////////////////


/////////////////////////////////////////////////////////////////////
//                 HIGH_LEVEL_CONTROL CLASS MEMBERS                //
/////////////////////////////////////////////////////////////////////
high_level_control::high_level_control() : Node("high_level_control") {

  // Publishers
  operation_command_publisher_ = this->create_publisher<std_msgs::msg::String>("operation_command", 10);

  // Subscriptions
  operation_status_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "operation_status", 10, std::bind(&high_level_control::operation_status_callback, this, std::placeholders::_1));
  
}

/////////////////////////////////////////////////////////////////////
//                      SUBSCRIPTION CALLBACKS                     //
/////////////////////////////////////////////////////////////////////

void high_level_control::operation_status_callback(std_msgs::msg::String status) {
  // TODO
  return;
}

/////////////////////////////////////////////////////////////////////
//                              MAIN                               //
/////////////////////////////////////////////////////////////////////

int main(int argc, char * argv[]){
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<high_level_control>());
  rclcpp::shutdown();
  return 0;
}