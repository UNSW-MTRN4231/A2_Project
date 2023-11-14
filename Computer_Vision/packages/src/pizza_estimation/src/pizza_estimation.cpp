#include "rclcpp/rclcpp.hpp"
#include "yolov8_msgs/msg/detection_array.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "geometry_msgs/msg/point.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_msgs/msg/float64.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
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
        operation_command_subscription_ = this->create_subscription<std_msgs::msg::String>(
            "/operation_command", 10, std::bind(&DetectionSubscriber::operation_command_callback, this, std::placeholders::_1));
        plate_aruco_marker_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "plate_aruco_marker", 10, std::bind(&DetectionSubscriber::plate_aruco_marker_callback, this, std::placeholders::_1));
        warp_pov_tf_subscription_ = this->create_subscription<std_msgs::msg::Float64MultiArray>(
            "warp_pov_tf", 10, std::bind(&DetectionSubscriber::warp_pov_tf_callback, this, std::placeholders::_1));
        pizza_aruco_marker_subscription_ = this->create_subscription<geometry_msgs::msg::Pose>(
            "pizza_aruco_marker", 10, std::bind(&DetectionSubscriber::pizza_aruco_marker_callback, this, std::placeholders::_1));

        image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_with_circles", 10);
        centroid_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("pizza_centroid", 10);
        radius_publisher_ = this->create_publisher<std_msgs::msg::Float64>("pizza_radius", 10);
        plate_centroid_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("plate_centroid", 10);
        pizza_angle_publisher_ = this->create_publisher<std_msgs::msg::Float64>("pizza_angle", 10);


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

    geometry_msgs::msg::Pose plate_pose_;
    cv::Mat warp_pov_tf_matrix_;

    geometry_msgs::msg::Pose pizza_pose_;
    double last_pizza_angle_;
    const double angle_change_threshold_ = 5.0 * (M_PI / 180.0); 

    void image_callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        current_image_ = cv_bridge::toCvCopy(msg, "bgr8")->image;
    }

    void operation_command_callback(const std_msgs::msg::String::SharedPtr msg)
    {
        if (msg->data != "Detect") {
            return; 
        }

        if (current_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image data yet.");
            return;
        }
        publish_poses_and_radii();
    }
    
    void callback(const yolov8_msgs::msg::DetectionArray::SharedPtr msg)
    {
        if (current_image_.empty()) {
            RCLCPP_WARN(this->get_logger(), "No image data yet.");
            return;
        }
        
        for (const auto& detection : msg->detections) {
            if (detection.class_id == 53) {
                std::tie(avg_radius, centroid_point, radius_end) = computeEstimation(detection.mask);
                if (isSignificantMovement(centroid_point, last_centroid_point_)) {
                    last_centroid_point_ = centroid_point;
                }
            }
        }
        centroid_msg.x = last_centroid_point_.y;
        centroid_msg.y = last_centroid_point_.x;
        radius_msg.data = avg_radius;
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

    void publish_poses_and_radii()
    {
        cv::Mat image_with_circle = current_image_.clone();
        cv::circle(image_with_circle, last_centroid_point_ + robot_arm_position_, 5, cv::Scalar(0, 0, 255), -1); // centroid
        cv::line(image_with_circle, last_centroid_point_ + robot_arm_position_, radius_end + robot_arm_position_, cv::Scalar(255, 0, 0), 2); // radius line

        auto image_msg = cv_bridge::CvImage(std_msgs::msg::Header(), "bgr8", image_with_circle).toImageMsg();
        image_publisher_->publish(*image_msg);
        centroid_publisher_->publish(centroid_msg);
        radius_publisher_->publish(radius_msg);
    }

        void warp_pov_tf_callback(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() == 9) {
            warp_pov_tf_matrix_ = cv::Mat(3, 3, CV_64F);
            memcpy(warp_pov_tf_matrix_.data, msg->data.data(), msg->data.size() * sizeof(double));
            if (plate_pose_.position.x != 0 || plate_pose_.position.y != 0) {
                publish_transformed_centroid(plate_pose_);
            }
        }
        else {
            RCLCPP_ERROR(this->get_logger(), "Received warp_pov_tf does not have 9 elements.");
        }
    }
    void plate_aruco_marker_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        plate_pose_ = *msg;
        if (!warp_pov_tf_matrix_.empty()) {
            publish_transformed_centroid(plate_pose_);
        }
    }
    cv::Point2f transform_plate_pose(const geometry_msgs::msg::Pose& plate_pose)
    {
        // Convert Pose to Point2f
        cv::Point2f plate_point(plate_pose.position.x, plate_pose.position.y);

        // Apply the perspective transformation
        std::vector<cv::Point2f> plate_points = {plate_point};
        std::vector<cv::Point2f> transformed_points;
        cv::perspectiveTransform(plate_points, transformed_points, warp_pov_tf_matrix_);

        if (!transformed_points.empty())
        {
            return transformed_points[0];
        }
        else
        {
            return cv::Point2f(-1.0f, -1.0f); // Return an invalid point if transformation fails
        }
    }
    void publish_transformed_centroid(const geometry_msgs::msg::Pose& plate_pose)
    {
        cv::Point2f transformed_point = transform_plate_pose(plate_pose);
        transformed_point.x -= robot_arm_position_.x;
        transformed_point.y -= robot_arm_position_.y;
        geometry_msgs::msg::Point centroid_msg;
        centroid_msg.x = transformed_point.x;
        centroid_msg.y = transformed_point.y;
        plate_centroid_publisher_->publish(centroid_msg);
    }

    void pizza_aruco_marker_callback(const geometry_msgs::msg::Pose::SharedPtr msg)
    {
        double current_angle = msg->orientation.w; // Assuming this is in radians
        double angle_change = std::abs(current_angle - last_pizza_angle_);

        if (angle_change > angle_change_threshold_) {
            std_msgs::msg::Float64 angle_msg;
            angle_msg.data = current_angle;
            last_pizza_angle_ = current_angle;
        }
        pizza_angle_publisher_->publish(angle_msg);
    }
    rclcpp::Subscription<yolov8_msgs::msg::DetectionArray>::SharedPtr subscription_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr image_subscription_;
    rclcpp::Subscription<std_msgs::msg::String>::SharedPtr operation_command_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr plate_aruco_marker_subscription_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr warp_pov_tf_subscription_;
    rclcpp::Subscription<geometry_msgs::msg::Pose>::SharedPtr pizza_aruco_marker_subscription_;

    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr image_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr centroid_publisher_;
    rclcpp::Publisher<geometry_msgs::msg::Point>::SharedPtr plate_centroid_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr radius_publisher_;
    rclcpp::Publisher<std_msgs::msg::Float64>::SharedPtr pizza_angle_publisher_;
};

int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<DetectionSubscriber>());
    rclcpp::shutdown();
    return 0;
}
