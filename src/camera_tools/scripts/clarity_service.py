#!/home/mark/ros2-humble-env/bin/python3

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image
from dom import DOM
from camera_tools_interfaces.srv import CalcClarity


class ClarityService(Node):
    def __init__(self):
        super().__init__('vision_controller')

        # Create the service
        self.service_ = self.create_service(CalcClarity,'calc_clarity',self.CalculateClarity)
        self.iqa = DOM()

        # Initialize OpenCV Bridge
        self.bridge = CvBridge()

    def CalculateClarity(self, request, response):
        self.get_logger().info('Received video frame')
        frame = self.bridge.imgmsg_to_cv2(request.image)
        response.clarity_value = self.iqa.get_sharpness(frame)
        self.get_logger().info(f'Calculated Clarity Value {response.clarity_value}')
        return response

def main(args=None):
    rclpy.init(args=args)
    node = ClarityService()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()