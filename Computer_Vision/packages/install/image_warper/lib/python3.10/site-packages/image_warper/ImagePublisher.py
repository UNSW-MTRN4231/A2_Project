import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class ImagePublisher(Node):

    def __init__(self):
        super().__init__('image_publisher')
        self.publisher_ = self.create_publisher(Image, 'image_raw', 10)
        self.timer = self.create_timer(0.5, self.publish_image)
        self.bridge = CvBridge()
        self.image_path = "/home/mtrn/4231/A2_Project/Computer_Vision/packages/src/image_warper/image_warper/1.jpeg" # Adjust the path to your image
        self.cv_image = cv2.imread(self.image_path)

    def publish_image(self):
        if self.cv_image is not None:
            image_msg = self.bridge.cv2_to_imgmsg(self.cv_image, 'bgr8')
            self.publisher_.publish(image_msg)
        else:
            self.get_logger().warn('Failed to read the image!')

def main(args=None):
    rclpy.init(args=args)
    image_publisher_node = ImagePublisher()
    rclpy.spin(image_publisher_node)
    image_publisher_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
