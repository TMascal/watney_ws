#!/home/mark/ros2-humble-env/bin/python3
# Created by Tim Mascal on 2/23/25

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
from camera_tools_interfaces.srv import TakePicture  # Update with your service definition


class TakePictureServiceServer(Node):
    def __init__(self):
        super().__init__('take_picture_service_server')
        # Create the service
        self.service_ = self.create_service(
            TakePicture,
            'take_picture',
            self.handle_request
        )
        # Initialize CvBridge for image conversion
        self.bridge = CvBridge()
        self.get_logger().info('Service server ready.')

    def handle_request(self, request, response):
        """Handle service requests"""
        # Open the camera
        cap = cv2.VideoCapture(2, cv2.CAP_V4L2)

        if not cap.isOpened():
            self.get_logger().error('Failed to open the camera.')
            return response

        # Set the desired resolution
        desired_width = 2592  # Hardcoded width
        desired_height = 1944  # Hardcoded height
        cap.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

        # Capture a frame
        ret, frame = cap.read()
        cap.release()  # Release the camera resource after capturing

        if not ret or frame is None:
            self.get_logger().error('Captured frame is empty.')
            return response

        # Convert the captured frame (OpenCV image) to ROS2 sensor_msgs/Image
        img_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
        response.image = img_msg

        self.get_logger().info('Image captured and returned.')
        return response


def main(args=None):
    rclpy.init(args=args)

    service_server = TakePictureServiceServer()
    try:
        rclpy.spin(service_server)
    except KeyboardInterrupt:
        pass
    finally:
        service_server.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()