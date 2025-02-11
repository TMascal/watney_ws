import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
import cv2
import numpy as np
import time

class BrightnessPublisher(Node):
    def __init__(self):
        super().__init__('brightness_publisher')
        self.publisher_ = self.create_publisher(Float32, 'image_brightness', 10)
        self.timer = self.create_timer(1.0, self.publish_brightness)  # Publish every 1 second
        self.cap = cv2.VideoCapture(0)
        time.sleep(0.5)  # Allow the camera to warm up
        self.get_logger().info('Brightness Publisher Node Initialized')

    def publish_brightness(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('Failed to capture frame from camera')
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        brightness = np.mean(gray)
        
        msg = Float32()
        msg.data = brightness
        self.publisher_.publish(msg)
        self.get_logger().info(f'Published brightness: {brightness}')

    def destroy_node(self):
        self.cap.release()  # Release the camera resource
        super().destroy_node()

def main(args=None):
    rclpy.init(args=args)
    node = BrightnessPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('Node stopped cleanly')
    except Exception as e:
        node.get_logger().error(f'Exception in node: {e}')
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()
