import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge

class ImageWarper(Node):

    def __init__(self):
        super().__init__('image_warper')
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'image_warped', 10)
        self.pov_publisher_ = self.create_publisher(Float64MultiArray, 'warp_pov_tf', 10)
        self.bridge = CvBridge()

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        purple_dots = get_purple_dots_coordinates(cv_image)
        sorted_purple_dots = sort_coordinates(purple_dots)

        if len(sorted_purple_dots) != 4:
            self.get_logger().warn('Did not detect exactly four purple dots.')
            return

        warped_image = self.warp_image(cv_image, sorted_purple_dots)
        img_msg = self.bridge.cv2_to_imgmsg(warped_image, 'bgr8')
        self.publisher_.publish(img_msg)

    def warp_image(self, img, purple_dots):
        src = np.array(purple_dots, dtype=np.float32)
        dst = np.array([[0, 0], [1000, 0], [0, 1000], [1000, 1000]], dtype=np.float32)
        M = cv2.getPerspectiveTransform(src, dst)

        # Publish the transformation matrix
        matrix_msg = Float64MultiArray()
        matrix_msg.data = [float(value) for value in M.ravel()]
        self.pov_publisher_.publish(matrix_msg)
        return cv2.warpPerspective(img, M, (1000, 1000))


def get_purple_dots_coordinates(image):
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    lower_purple = np.array([180 * 255 / 360, 90, 90])
    upper_purple = np.array([220 * 255 / 360, 255, 255])
    mask = cv2.inRange(hsv, lower_purple, upper_purple)
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    coordinates = []
    for contour in contours:
        if 100 < cv2.contourArea(contour) < 10000:
            M = cv2.moments(contour)
            if M["m00"]:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                coordinates.append((cX, cY))
    return coordinates


def sort_coordinates(coordinates):
    sorted_coordinates = sorted(coordinates, key=lambda coord: coord[0])
    left = sorted_coordinates[:2]
    right = sorted_coordinates[2:]
    return [sorted(left, key=lambda coord: coord[1])[0], sorted(right, key=lambda coord: coord[1])[0],
            sorted(left, key=lambda coord: coord[1])[1], sorted(right, key=lambda coord: coord[1])[1]]


def main(args=None):
    rclpy.init(args=args)
    image_warper_node = ImageWarper()
    rclpy.spin(image_warper_node)
    image_warper_node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()