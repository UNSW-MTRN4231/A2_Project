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
  operation_status_publisher_ = this->create_publisher<std_msgs::msg::String>("operation_status", 10);
  operation_command_publisher_ = this->create_publisher<std_msgs::msg::String>("operation_command", 10);

  // Subscriptions
  pizza_radius_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "pizza_radius", 10, std::bind(&high_level_control::pizza_radius_callback, this, std::placeholders::_1));
  pizza_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "pizza_pose", 10, std::bind(&high_level_control::pizza_pose_callback, this, std::placeholders::_1));
  operation_status_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "operation_status", 10, std::bind(&high_level_control::operation_status_callback, this, std::placeholders::_1));
  
}

// Checks if pizza radius and pose are set, and publishes a message to /operation_status
void high_level_control::update_detection_status() {
  if (pizza_radius_is_set && pizza_pose_is_set) {
    detection_is_complete = true;

    // Publish message
    std_msgs::msg::String msg;
    msg.data = "Detection Complete";
    operation_status_publisher_->publish(msg);
  }
}

/////////////////////////////////////////////////////////////////////
//                      SUBSCRIPTION CALLBACKS                     //
/////////////////////////////////////////////////////////////////////

void high_level_control::pizza_radius_callback(std_msgs::msg::Float32 radius) {
  pizza_radius_is_set = true;
  update_detection_status();
  return;
}

void high_level_control::pizza_pose_callback(geometry_msgs::msg::Pose pose) {
  pizza_pose_is_set = true;
  update_detection_status();
  return;
}

void high_level_control::operation_status_callback(std_msgs::msg::String status) {
  if (status.data == "Planning Complete") {
    // TODO wait for key press here
    std_msgs::msg::String msg;
    msg.data = "Cut";
    operation_command_publisher_->publish(msg);
  }

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