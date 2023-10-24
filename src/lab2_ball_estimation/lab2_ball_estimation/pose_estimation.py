import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from cv_bridge import CvBridge
import cv2
import math
import numpy as np

class BallDetectionNode(Node):
    def __init__(self):
        super().__init__('ball_detection_node')
        self.subscription = self.create_subscription( Image, 'image_raw', self.image_callback, 10 )
        self.pose_publisher = self.create_publisher(PoseStamped, 'ball_pose', 10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        pose = self.detect_ball(cv_image)
        self.get_logger().info('Ball Pose: {}'.format(pose))
        if pose:
            pose_stamped = PoseStamped()
            pose_stamped.header.stamp = self.get_clock().now().to_msg()
            pose_stamped.header.frame_id = 'camera_frame'
            pose_stamped.pose.position.x = pose[0]
            pose_stamped.pose.position.y = pose[1]
            pose_stamped.pose.position.z = pose[2]
            self.pose_publisher.publish(pose_stamped)

    def detect_ball(self, image):
        # Convert BGR to HSV
        hsv_image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # Define the range of red color in HSV
        lower_red = np.array([0, 100, 100])
        upper_red = np.array([10, 255, 255])

        # Threshold the HSV image to get only red colors
        mask = cv2.inRange(hsv_image, lower_red, upper_red)

        # Perform morphological operations to remove noise and fill gaps
        kernel_small = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (3, 3))
        kernel_large = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (7, 7))

        mask = cv2.erode(mask, kernel_small, iterations=1)
        mask = cv2.dilate(mask, kernel_large, iterations=5)
        mask = cv2.erode(mask, kernel_small, iterations=1)

        # Perform closing to close gaps in the contour
        #mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel_large)

        # Print the red mask
        cv2.imshow("Red Mask", mask)
        cv2.waitKey(1)

        # Find contours of the red regions
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Filter contours based on area and circularity
        filtered_contours = []
        for contour in contours:
            area = cv2.contourArea(contour)
            perimeter = cv2.arcLength(contour, True)
            if perimeter > 0:
                circularity = 4 * np.pi * (area / (perimeter * perimeter))
                if 800 < area < 25000 and circularity > 0.7:
                    filtered_contours.append(contour)

        # Find the centroid of the largest contour
        if filtered_contours:
            largest_contour = max(filtered_contours, key=cv2.contourArea)
            moments = cv2.moments(largest_contour)
            cx = int(moments['m10'] / moments['m00'])
            cy = int(moments['m01'] / moments['m00'])

            # Draw a circle around the detected ball
            x, y, w, h = cv2.boundingRect(largest_contour)

            # Width and height of the contour
            width = w
            height = h

            distance = -0.0075 * (width + height)/2 + 1.45

            # Estimate the pose of the ball (example values)
            azimuth_degrees = (cx - image.shape[1] / 2) * (25/320)
            elevation_degrees = (cy - image.shape[0] / 2) * (20/240)


            # Convert angles to radians
            azimuth_radians = math.radians(azimuth_degrees)
            elevation_radians = math.radians(elevation_degrees)

            # Calculate the X, Y, and Z coordinates
            x = distance * math.cos(elevation_radians) * math.cos(azimuth_radians)
            y = distance * math.cos(elevation_radians) * math.sin(azimuth_radians)
            z = distance * math.sin(elevation_radians)

            return (x, y, z)
        else:
            return None

def main(args=None):
    rclpy.init(args=args)
    node = BallDetectionNode()
    rclpy.spin(node)
    rclpy.shutdown()

if __name__ == '__main__':
    main()