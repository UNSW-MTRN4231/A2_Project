#include <rclcpp/rclcpp.hpp>
#include "tf2/LinearMath/Quaternion.h"
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <std_msgs/msg/string.hpp>
#include <cmath>
#include <memory>
#include <thread>
#include <chrono>
#include <functional>
#include <string>
#include <tf2_ros/transform_broadcaster.h>

using namespace std::chrono_literals;
using std::placeholders::_1;

class DynamicTFBroadcaster : public rclcpp::Node
{
  public:
    DynamicTFBroadcaster()
    : Node("DynamicTFBroadcaster")
    {
      subscription_ = this->create_subscription<geometry_msgs::msg::PoseStamped>( "ball_pose", 10, std::bind(&DynamicTFBroadcaster::topic_callback, this, _1));
      tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);
    }

  private:
    void topic_callback(const geometry_msgs::msg::PoseStamped & msg) const
    {
      RCLCPP_INFO(this->get_logger(), "I heard: '%f'", msg.pose.position.x);

      geometry_msgs::msg::TransformStamped transform_msg;
      tf_broadcaster_->sendTransform(transform_msg);
    }


    rclcpp::Subscription<geometry_msgs::msg::PoseStamped>::SharedPtr subscription_;
    rclcpp::Publisher<visualization_msgs::msg::Marker>::SharedPtr publisher_;
    std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};



int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DynamicTFBroadcaster>());
  rclcpp::shutdown();
  return 0;
}