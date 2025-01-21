#!/home/mark/ros2-humble-env/bin/python3

# Make sure to use shebang #! instead of a comment # !
# Created by Tim Mascal on Jan 16, 2025
# Simple ROS 2 'Hello World' script

import rclpy
from rclpy.node import Node


class HelloWorldNode(Node):
    def __init__(self):
        super().__init__('hello_world_node')  # Node name
        self.get_logger().info('Hello, World! This is a ROS 2 node.')

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client
    node = HelloWorldNode()
    rclpy.spin(node)  # Keep the node running to process callbacks
    rclpy.shutdown()  # Shutdown ROS 2


# Run the main function
if __name__ == '__main__':
    main()

