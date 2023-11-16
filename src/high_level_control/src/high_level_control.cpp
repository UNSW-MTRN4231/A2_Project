#include "high_level_control.hpp"

/////////////////////////////////////////////////////////////////////
//                              HELPERS                            //
/////////////////////////////////////////////////////////////////////

// Sleeps thread for a given number of seconds
void wait(int t) {
  std::chrono::seconds duration(t);
  std::this_thread::sleep_for(duration);
  return;
}

/////////////////////////////////////////////////////////////////////
//                 HIGH_LEVEL_CONTROL CLASS MEMBERS                //
/////////////////////////////////////////////////////////////////////
high_level_control::high_level_control() : Node("high_level_control") {

  // Publishers
  operation_command_publisher_ = this->create_publisher<std_msgs::msg::String>("operation_command", 10);
  operation_status_publisher_ = this->create_publisher<std_msgs::msg::String>("operation_status", 10);
  gui_text_publisher_ = this->create_publisher<std_msgs::msg::String>("gui_text", 10);

  // Add callbacks to subscription options
  auto sub_options_keyboard = rclcpp::SubscriptionOptions();
  sub_options_keyboard.callback_group = keyboard_cb_group;
  auto sub_options_status = rclcpp::SubscriptionOptions();
  sub_options_status.callback_group = status_cb_group;

  // Subscriptions
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
  operation_sequence.push_back("Plan Trajectories");
  operation_sequence.push_back("Pick Cutting Tool");
  operation_sequence.push_back("Detect");
  operation_sequence.push_back("Cut");
  operation_sequence.push_back("Detect");
  operation_sequence.push_back("Place Cutting Tool");
  operation_sequence.push_back("Detect");
  operation_sequence.push_back("Pick Serving Tool");
  operation_sequence.push_back("Detect");

  for (int i=0; i<num_slices; i++){
    operation_sequence.push_back("Pick Slice");
    operation_sequence.push_back("Detect");
    operation_sequence.push_back("Place Slice");
    operation_sequence.push_back("Detect");
  }

  operation_sequence.push_back("Place Serving Tool");

  operation_sequence_mutex.unlock();
}

void high_level_control::execute_next_operation() {
  std::lock_guard<std::mutex> lock(operation_sequence_mutex);

  // Publish gui text
  std_msgs::msg::String msg;
  msg.data = "Current operation: " + operation_sequence.at(0);
  gui_text_publisher_->publish(msg);

  wait(2);

  // Publish operation command
  msg.data = operation_sequence.at(0);
  operation_command_publisher_->publish(msg);

  operation_sequence_mutex.unlock();
}

/////////////////////////////////////////////////////////////////////
//                      SUBSCRIPTION CALLBACKS                     //
/////////////////////////////////////////////////////////////////////

void high_level_control::operation_status_callback(std_msgs::msg::String status) {

  std::lock_guard<std::mutex> lock(operation_sequence_mutex);

  // Check if status ends with "Fail"
  bool complete = true;
  if (status.data.length() >= 4 && status.data.substr(status.data.length() - 4) == "Fail") {
    complete = false;
  }

  // Take action depending on complete/fail
  std_msgs::msg::String msg;
  if (complete) {
    msg.data = operation_sequence.at(0) + " Complete. Press [space] to continue";
    // Remove completed operation from sequence
    operation_sequence.erase(operation_sequence.begin());
  } else {
    msg.data = operation_sequence.at(0) + " Fail. Move objects and press [space] to continue";
    // Insert detection to allow operator to move objects
    operation_sequence.insert(operation_sequence.begin(), "Detect");
  }
  gui_text_publisher_->publish(msg);


  if (operation_sequence.empty()) {
    // Shutdown if sequence is complete
    RCLCPP_INFO(this->get_logger(), "Operation sequence complete");

    // Publish gui text
    msg.data = "Operation sequence complete, it's time to eat :)";
    gui_text_publisher_->publish(msg);
    rclcpp::shutdown();
  }

  operation_sequence_mutex.unlock();

  previous_operation_complete = true;

  return;
}

void high_level_control::keyboard_input_callback(std_msgs::msg::String key) {
  // Shutdown
  if (key.data == "q") {
    rclcpp::shutdown();
  }

  // Next step
  if (key.data == "space") {
    if (previous_operation_complete) {
      previous_operation_complete = false;
      execute_next_operation();
    } else {
      RCLCPP_WARN(this->get_logger(), "Cannot proceed - previous operation not completed");
    }
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