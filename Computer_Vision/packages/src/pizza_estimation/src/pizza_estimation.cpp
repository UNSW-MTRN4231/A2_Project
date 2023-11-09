#include "rclcpp/rclcpp.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/float64.hpp"  
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <vector>
#include <numeric>

class DetectionSubscriber : public rclcpp::Node
{
public:
    DetectionSubscriber()
    : Node("detection_subscriber")
    {
        subscription_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
            "/yolo/old_detections", 10, std::bind(&DetectionSubscriber::callback, this, std::placeholders::_1));
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_warped", 10, std::bind(&DetectionSubscriber::image_callback, this, std::placeholders::_1));

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_with_circles", 10);
        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("pizza_centroid", 10);
        radius_publisher_ = this->create_publisher<std_msgs::msg::Float64>("pizza_radius", 10);

        last_centroid_point_ = cv::Point(-1, -1);
        robot_arm_position_ = cv::Point(-228, 58); // corner0 in base frame of robot: x = -228mm, y = -58mm, z = 0mm
    }

private:
    cv::Mat current_image_;
    cv::Point robot_arm_position_;
    cv::Point centroid_point, radius_end, last_centroid_point_;
    geometry_msgs::msg::Point centroid_msg;
    std_msgs::msg::Float64 radius_msg;
    double avg_radius;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }
    
    void callback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg)
    {
        if (current_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image data yet.");
            return;
        }
        
        cv::Mat image_with_circle = current_image_.clone();
        for (const auto& detection : msg->detections) {
            if (detection.class_id == 53) {
                std::tie(avg_radius, centroid_point, radius_end) = computeEstimation(detection.mask);

                cv::circle(image_with_circle, centroid_point+robot_arm_position_, 5, cv::Scalar(0, 0, 255), -1); // centroid
                cv::line(image_with_circle, centroid_point+robot_arm_position_, radius_end, cv::Scalar(255, 0, 0), 2); // radius line

                if (isSignificantMovement(centroid_point, last_centroid_point_)) {
                    last_centroid_point_ = centroid_point;

                    centroid_msg.x = centroid_point.y;
                    centroid_msg.y = centroid_point.x;
                    centroid_msg.z = 0;
                    centroid_publisher_->publish(centroid_msg);

                    radius_msg.data = avg_radius;
                    radius_publisher_->publish(radius_msg);
                }

            }
        }
        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_with_circle).toImageMsg();
        image_publisher_->publish(*image_msg);
    }
    
    std::pair<float, float> computeCentroid(const yolov8_msgs::msg::Mask& mask) const
    {
        float sum_x = 0;
        float sum_y = 0;
        int count = 0;
        
        for (const auto& point : mask.data) {
            sum_x += point.x;
            sum_y += point.y;
            ++count;
        }
        
        return {sum_x / count, sum_y / count};
    }
    std::tuple<double, cv::Point, cv::Point> computeEstimation(const yolov8_msgs::msg::Mask& mask) const
    {
        std::vector<cv::Point> points;
        for (const auto& point : mask.data) {
            points.emplace_back(point.x, point.y);
        }
        cv::Rect bounding_rect = cv::boundingRect(points);
        cv::Point center = (bounding_rect.tl() + bounding_rect.br()) / 2;
        cv::Point top_center(bounding_rect.x + bounding_rect.width / 2, bounding_rect.y);
        double avg_radius = cv::norm(top_center - center);
        return std::make_tuple(avg_radius, center - robot_arm_position_, top_center);
    }

    bool isSignificantMovement(const cv::Point& current, const cv::Point& last) const
    {
        if (last.x == -1 && last.y == -1) return true;
        double distance = cv::norm(current - last);
        return distance > 20.0; // Check if the movement is more than 20mm
    }
    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr radius_publisher_;
};



int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionSubscriber>());
    rclcpp::shutdown();
    return 0;
}
