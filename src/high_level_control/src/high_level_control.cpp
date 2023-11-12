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
  operation_status_publisher_ = this->create_publisher<std_msgs::msg::String>("operation_status", 10);

  // Add callbacks to subscription options
  auto sub_options_keyboard = rclcpp::SubscriptionOptions();
  sub_options_keyboard.callback_group = keyboard_cb_group;
  auto sub_options_status = rclcpp::SubscriptionOptions();
  sub_options_status.callback_group = status_cb_group;

  // Subscriptions
  pizza_radius_subscription_ = this->create_subscription<std_msgs::msg::Float32>(
    "pizza_radius", 10, std::bind(&high_level_control::pizza_radius_callback, this, std::placeholders::_1),sub_options_status);
  pizza_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "pizza_pose", 10, std::bind(&high_level_control::pizza_pose_callback, this, std::placeholders::_1),sub_options_status);
  tool_jig_pose_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
    "tool_jig_pose", 10, std::bind(&high_level_control::tool_jig_pose_callback, this, std::placeholders::_1),sub_options_status);
  operation_status_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "operation_status", 10, std::bind(&high_level_control::operation_status_callback, this, std::placeholders::_1),sub_options_status);
  keyboard_input_subscription_ = this->create_subscription<std_msgs::msg::String>(
    "keyboard_input", 10, std::bind(&high_level_control::keyboard_input_callback, this, std::placeholders::_1),sub_options_keyboard);

  // Generate operation sequence
  generate_operation_sequence();
}

void high_level_control::generate_operation_sequence() {
  // TODO modify
  std::lock_guard<std::mutex> lock(operation_sequence_mutex);
  operation_sequence.push_back("Detect");
  operation_sequence.push_back("Pick Cutting Tool");
  operation_sequence.push_back("Plan Trajectories");
  operation_sequence.push_back("Cut");
  operation_sequence.push_back("Pick Serving Tool");
  operation_sequence.push_back("Pick Slice");
  operation_sequence_mutex.unlock();
}

void high_level_control::execute_next_operation() {
  std::lock_guard<std::mutex> lock(operation_sequence_mutex);
  std_msgs::msg::String msg;
  msg.data = operation_sequence.at(0);
  operation_command_publisher_->publish(msg);
  operation_sequence_mutex.unlock();
}

// Checks if pizza radius and pose are set, and publishes a message to /operation_status
void high_level_control::update_detection_status() {
  if (pizza_radius_is_set && pizza_pose_is_set && tool_jig_pose_is_set && !detection_is_complete) {
    detection_is_complete = true;

    // Publish message // TODO get computer vision to do this
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

void high_level_control::tool_jig_pose_callback(geometry_msgs::msg::Pose pose) {
  tool_jig_pose_is_set = true;
  update_detection_status();
  return;
}

void high_level_control::operation_status_callback(std_msgs::msg::String status) {

  // Remove completed operation from the operation sequence
  std::lock_guard<std::mutex> lock(operation_sequence_mutex);
  operation_sequence.erase(operation_sequence.begin());
  bool operation_sequence_complete = operation_sequence.empty();
  operation_sequence_mutex.unlock();

  if (operation_sequence_complete) {
    RCLCPP_INFO(this->get_logger(), "Operation sequence complete!!!");
  } else {
    // Prompt for next operation
    std::string prompt_str = status.data + ". Press space to continue.";
    RCLCPP_INFO(this->get_logger(), "%s", prompt_str.c_str());
  }

  return;
}

void high_level_control::keyboard_input_callback(std_msgs::msg::String key) {
  // Shutdown
  if (key.data == "q") {
    rclcpp::shutdown();
  }

  // Next step
  if (key.data == "space") {
    execute_next_operation();
  }
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