# plant_cv/scripts/image_clarity_analyzer.py

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2


class ImageClarityAnalyzer(Node):
    def __init__(self):
        super().__init__('image_clarity_analyzer')
        self.subscription = self.create_subscription(
            Image,
            'camera/image_raw',
            self.image_callback,
            10)
        self.bridge = CvBridge()

    def image_callback(self, msg):
        # Convert ROS Image message to OpenCV image
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

        # Calculate the image clarity
        clarity_value = self.analyze_clarity(frame)

        # Print the clarity value
        self.get_logger().info(f'Image clarity value: {clarity_value}')

    def analyze_clarity(self, image):
        # Convert to grayscale
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)

        # Use the Laplacian method to calculate clarity
        return cv2.Laplacian(gray, cv2.CV_64F).var()


def main(args=None):
    rclpy.init(args=args)
    node = ImageClarityAnalyzer()
    rclpy.spin(node)

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()