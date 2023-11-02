import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
from cv_bridge import CvBridge
# from geometry_msgs.msg import Pose

class ArucoDetector(Node):

    def __init__(self):
        super().__init__('aruco_detector')
        self.subscription = self.create_subscription(Image, 'image_raw', self.listener_callback, 10)
        self.publisher_ = self.create_publisher(Image, 'image_aruco_detected', 10) # TO CHANGE TO CORRECT TYPE
        self.startingAngles = -10
        self.angleChange = 0
        # self.pov_publisher_ = self.create_publisher(Float64MultiArray, 'warp_pov_tf', 10)
        # self.labelled_publisher_ = self.create_publisher(Image, 'image_labelled', 10)# New publisher
        #         image_publisher_ = this->create_publisher<sensor_msgs::msg::Image>("image_with_circles", 10);
        # centroid_publisher_ = this->create_publisher<geometry_msgs::msg::Point>("pizza_centroid", 10);
        # radius_publisher_ = this->create_publisher<std_msgs::msg::Float64>("pizza_radius", 10);

        self.bridge = CvBridge()
        self.prev_M = None

    def listener_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        arucoDict = cv2.aruco.Dictionary_get(cv2.aruco.DICT_4X4_50)
        arucoParams = cv2.aruco.DetectorParameters_create()
        (corners, ids, rejected) = cv2.aruco.detectMarkers(cv_image, arucoDict,parameters=arucoParams)

        labelled_image = cv_image.copy()

        # Corner detection and labelling 
        # https://pyimagesearch.com/2020/12/21/detecting-aruco-markers-with-opencv-and-python/
        if len(corners) > 0:
            # flatten the ArUco IDs list
            ids = ids.flatten()
            # loop over the detected ArUCo corners
            for (markerCorner, markerID) in zip(corners, ids):
                # extract the marker corners (which are always returned in
                # top-left, top-right, bottom-right, and bottom-left order)
                corners = markerCorner.reshape((4, 2))
                (topLeft, topRight, bottomRight, bottomLeft) = corners
                # convert each of the (x, y)-coordinate pairs to integers

                # note: these are relative to the original marker designs.
                topRight = (int(topRight[0]), int(topRight[1]))
                bottomRight = (int(bottomRight[0]), int(bottomRight[1]))
                bottomLeft = (int(bottomLeft[0]), int(bottomLeft[1]))
                topLeft = (int(topLeft[0]), int(topLeft[1]))
                
                # ASSUMPTION: WE KNOW EXACTLY WHICH ARUCO MARKERS WE WORK WITH
                if markerID == 0:
                    self.angleChange = np.arctan2((topRight[1]-topLeft[1]),(topRight[0]-topLeft[0]))
                    self.get_logger().info("Angle change recorded: " + str(self.angleChange))
                    #initialising first marker detection and its orientation
                    if self.startingAngle == -10:
                        self.startingAngle = self.angleChange
                        self.angleChange = 0
                        self.get_logger().info("Initialised starting angles: " + str(self.startingAngle))
                    
                

                cv2.line(labelled_image, topLeft, topRight, (0, 255, 0), 2)
                cv2.line(labelled_image, topRight, bottomRight, (0, 255, 0), 2)
                cv2.line(labelled_image, bottomRight, bottomLeft, (0, 255, 0), 2)
                cv2.line(labelled_image, bottomLeft, topLeft, (0, 255, 0), 2)

                cX = int((topLeft[0] + bottomRight[0]) / 2.0)
                cY = int((topLeft[1] + bottomRight[1]) / 2.0)
                cv2.circle(labelled_image, (cX, cY), 4, (0, 0, 255), -1)
                # draw the ArUco marker ID on the image
                cv2.putText(labelled_image, str(markerID),(topLeft[0], topLeft[1] - 15), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

        labelled_img_msg = self.bridge.cv2_to_imgmsg(labelled_image, 'bgr8')
        self.publisher_.publish(labelled_img_msg)
        

        # self.publisher_.publish(img_msg)
 
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
    aruco_detector_node = ArucoDetector()
    rclpy.spin(aruco_detector_node)
    aruco_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
