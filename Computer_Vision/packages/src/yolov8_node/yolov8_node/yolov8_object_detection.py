import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
import cv2
import numpy as np
import torch
from cv_bridge import CvBridge

PIZZA_CLASS_ID = 53
PLATE_CLASS_ID = 45

class YOLOv8Node(Node):
    def __init__(self):
        super().__init__('yolov8_node')
        
        # Initialize CvBridge
        self.bridge = CvBridge()

        # Load YOLOv8 model
        self.model = torch.jit.load("models/yolov8m-seg.pt")
        
        # Create Subscriber and Publisher
        self.subscription = self.create_subscription(
            Image, '/image_warped',
            self.listener_callback, 10)
        self.subscription  # prevent unused variable warning

        self.publisher = self.create_publisher(
            Image, '/yolo_detections', 10)
        
    def listener_callback(self, msg):
        # Convert ROS Image message to OpenCV format
        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        
        # Perform object detection
        detections = self.yolov8_detect(cv_image)
        
        # Convert detections to an image (for demonstration purposes)
        detection_image = self.detections_to_image(cv_image, detections)
        
        # Convert back to ROS Image message and publish
        img_msg = self.bridge.cv2_to_imgmsg(detection_image, 'bgr8')
        self.publisher.publish(img_msg)

    def yolov8_detect(self, image):
        # Resize image to (640, 640)
        resized_image = cv2.resize(image, (640, 640))

        # Convert to PyTorch tensor
        input_tensor = torch.tensor(resized_image, dtype=torch.float32).permute(2, 0, 1).unsqueeze(0)  # CxHxW and add batch dimension

        # Run model
        with torch.no_grad():
            detections = self.model(input_tensor)

        # Post-process the output
        # ... (this would depend on your specific YOLOv8 model)
        if isinstance(detections, torch.Tensor):
            detections = detections.cpu().numpy()
    
        # Threshold the detections (if necessary)
        # For example, you might set all values below 0.5 to zero
        # detections[detections < 0.5] = 0
        
        # Convert to class IDs (if necessary)
        # For example, you might round each value to the nearest integer
        # to get the class ID for each pixel
        class_ids = np.round(detections)
        
        return class_ids


    def detections_to_image(self, image, detections):
        # Assuming 'detections' is a 2D NumPy array representing a segmentation mask
        
        # Find pixels that belong to the class representing "pizza" (e.g., class ID = 1)
        pizza_mask = detections == PIZZA_CLASS_ID
        
        # Find pixels that belong to the class representing "plate" (e.g., class ID = 2)
        plate_mask = detections == PLATE_CLASS_ID

        # Draw the masks on the image (random values)
        image[pizza_mask] = [0, 255, 0]  # Green for pizza
        image[plate_mask] = [0, 0, 255]  # Red for plate

        return image


def main(args=None):
    rclpy.init(args=args)
    yolo_node = YOLOv8Node()
    rclpy.spin(yolo_node)
    yolo_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
