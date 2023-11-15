import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
from cv_bridge import CvBridge
from geometry_msgs.msg import Pose,PoseArray

class ImageWarper(Node):

    def __init__(self):
        super().__init__('image_warper')
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.aruco_subscription = self.create_subscription(PoseArray, 'markers_pose', self.aruco_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'image_warped', 10)
        self.pov_publisher_ = self.create_publisher(Float64MultiArray, 'warp_pov_tf', 10)
        self.labelled_publisher_ = self.create_publisher(Image, 'image_labelled', 10)# New publisher

        self.bridge = CvBridge()
        self.prev_M = None
        self.tag0Pose = Pose()
        self.tag1Pose = Pose()
        self.tag2Pose = Pose()
        self.tag3Pose = Pose()



    def aruco_callback(self,msg):
        [self.tag0Pose,self.tag1Pose,self.tag2Pose,self.tag3Pose] = msg.poses


    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        if not self.tag0Pose.position.x:
            # empty pose. Therefore publish original image
            self.get_logger().warn("tag not found")
            self.publisher_.publish(msg)


        purple_dots = [[int(self.tag0Pose.position.x),int(self.tag0Pose.position.y)],
                       [int(self.tag1Pose.position.x),int(self.tag1Pose.position.y)],
                       [int(self.tag2Pose.position.x),int(self.tag2Pose.position.y)],
                       [int(self.tag3Pose.position.x),int(self.tag3Pose.position.y)]] 

        self.get_logger().info("Pruple dots: " + str(len(purple_dots)))



        # purple_dots = get_purple_dots_coordinates(cv_image)

        if len(purple_dots) != 4:
            self.get_logger().warn('Did not detect exactly four purple dots.')
            if self.prev_M is not None:
                M = self.prev_M
            else:
                return
        else:
            sorted_purple_dots = sort_coordinates(purple_dots)
            M = self.get_perspective_transform(sorted_purple_dots)
            self.prev_M = M
            
            labelled_image = cv_image.copy()
            for index, (x, y) in enumerate(sorted_purple_dots):
                cv2.putText(labelled_image, str(index), (x, y), cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
            labelled_img_msg = self.bridge.cv2_to_imgmsg(labelled_image, 'bgr8')
            self.labelled_publisher_.publish(labelled_img_msg)

        warped_image = self.warp_image(cv_image, M)
        img_msg = self.bridge.cv2_to_imgmsg(warped_image, 'bgr8')
        self.publisher_.publish(img_msg)

    def get_perspective_transform(self, purple_dots):
        src = np.array(purple_dots, dtype=np.float32)
        src = np.array(purple_dots, dtype=np.float32)
        dst = np.array([[0, 0], [575, 0], [0, 760], [575, 760]], dtype=np.float32)
        # corner1 in base frame of robot: x = -231mm, y = -518mm, z = 0mm
        return cv2.getPerspectiveTransform(src, dst)

    def warp_image(self, img, M):
        matrix_msg = Float64MultiArray()
        matrix_msg.data = [float(value) for value in M.ravel()]
        self.pov_publisher_.publish(matrix_msg)
        return cv2.warpPerspective(img, M, (575, 760))

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
        if 10 < cv2.contourArea(contour) < 10000:
            M = cv2.moments(contour)
            if M["m00"]:
                cX = int(M["m10"] / M["m00"])
                cY = int(M["m01"] / M["m00"])
                coordinates.append((cX, cY))
    return coordinates

def sort_coordinates(coordinates):
    sorted_coordinates = sorted(coordinates, key=lambda coord: coord[0])
    left = sorted_coordinates[2:]
    right = sorted_coordinates[:2]
    return [sorted(right, key=lambda coord: coord[1])[1], sorted(right, key=lambda coord: coord[1])[0],
        sorted(left, key=lambda coord: coord[1])[1], sorted(left, key=lambda coord: coord[1])[0]]

# def compute_homography_to_robot_base(src_points):
#     src_points = np.array(src_points, dtype=np.float32)  # Ensure it's a numpy array
#     dst_robot = np.array([
#         [-50, -235],
#         [525, -235],
#         [-50, -950],
#         [525, -950],


#     ], dtype=np.float32)
    
#     H, _ = cv2.findHomography(src_points, dst_robot)
#     return H

def main(args=None):
    rclpy.init(args=args)
    image_warper_node = ImageWarper()
    rclpy.spin(image_warper_node)
    image_warper_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
