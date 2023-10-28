#include "rclcpp/rclcpp.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"  // Include to publish centroid
#include "std_msgs/msg/float64_multi_array.hpp"  // Homography matrix message type
#include "cv_bridge/cv_bridge.h"
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/calib3d/calib3d.hpp>

class DetectionSubscriber : public rclcpp::Node
{
public:
    DetectionSubscriber()
    : Node("detection_subscriber")
    {
        subscription_ = this->create_subscription<yolov8_msgs::msg::DetectionArray>(
            "/yolo/old_detections", 10, std::bind(&DetectionSubscriber::callback, this, std::placeholders::_1));
        image_subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "image_raw", 10, std::bind(&DetectionSubscriber::image_callback, this, std::placeholders::_1));
        homography_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "homography_matrix", 10, std::bind(&DetectionSubscriber::homography_callback, this, std::placeholders::_1));
        
        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_with_circles", 10);
        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("transformed_centroid", 10);
    }

private:
    cv::Mat current_image_;
    cv::Mat homography_matrix_;

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    void homography_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        // Assume it's a 3x3 matrix for the homography
        homography_matrix_ = cv::Mat(3, 3, CV_64F, msg->data.data());
    }

    void callback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg)
    {
        if (current_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image data yet.");
            return;
        }
        
        cv::Mat image_with_circle = current_image_.clone();
        
        for (const auto& detection : msg->detections) {
            auto mask_centroid = computeCentroid(detection.mask);
            cv::Point center(mask_centroid.first, mask_centroid.second);
            cv::circle(image_with_circle, center, 5, cv::Scalar(255, 0, 0), -1);
            
            // Apply the homography matrix if available
            if (!homography_matrix_.empty()) {
                std::vector<cv::Point2f> original_points = {center};
                std::vector<cv::Point2f> transformed_points;
                cv::perspectiveTransform(original_points, transformed_points, homography_matrix_);

                geometry_msgs::msg::Point transformed_centroid_msg;
                transformed_centroid_msg.x = transformed_points[0].x;
                transformed_centroid_msg.y = transformed_points[0].y;
                centroid_publisher_->publish(transformed_centroid_msg);
            }

            RCLCPP_INFO(this->get_logger(), "Mask Centroid: (%f, %f)", mask_centroid.first, mask_centroid.second);
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

    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr homography_subscription_;
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionSubscriber>());
    rclcpp::shutdown();
    return 0;
}
