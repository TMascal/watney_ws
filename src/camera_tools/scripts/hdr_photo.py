#!/home/mark/ros2-humble-env/bin/python3

# Make sure to use shebang #! instead of a comment # !
# Created by Tim Mascal on Jan 16, 2025
# Simple ROS 2 'Hello World' script

import rclpy
from rclpy.node import Node

# noinspection PyUnresolvedReferences
from camera_tools_interfaces.srv import ChangeExposure


class HDRNode(Node):
    def __init__(self):
        super().__init__('hdr_server_node')  # Node name
        self.get_logger().info('Starting HDR Server node.')

        # Create Object to Calls ROS2 Service to Change Cam Exposure
        self.client = self.create_client(ChangeExposure, 'change_exposure')
        while not self.client.wait_for_service(timeout_sec=1.0):
            self.get_logger().warn('The "change_exposure" service is not available yet.')

        # Prepare Service Message Object
        self.exposure_request = ChangeExposure.Request()
        self.exposure_request.exposure_value = 100 # Exposure Value is int64 in the ROS2 Service Definition

        self.get_logger().info(f'Sending request: a={self.exposure_request.exposure_value}')
        response = self.client.call(self.exposure_request)  # Synchronous service call

        self.get_logger().info(response)

        # Process the response
        if response:
            self.get_logger().info(f'Received response')

        else:
            self.get_logger().error('Failed to receive a response')

def main(args=None):
    rclpy.init(args=args)  # Initialize the ROS 2 Python client
    node = HDRNode()
    rclpy.spin(node)  # Keep the node running to process callbacks
    rclpy.shutdown()  # Shutdown ROS 2


# Run the main function
if __name__ == '__main__':
    main()

