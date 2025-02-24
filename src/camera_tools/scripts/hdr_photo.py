#!/home/mark/ros2-humble-env/bin/python3
from urllib import response

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node

import time

from camera_tools_interfaces.srv import ChangeExposure, TakePicture

class MyServiceClientNode(Node):
    def __init__(self):
        super().__init__('hdr_photo_node')

        # Create CV2 Bridge Object
        self.bridge = CvBridge()

        # Create the service clients
        self.client_1 = self.create_client(ChangeExposure, '/change_exposure')
        self.client_2 = self.create_client(TakePicture, '/take_picture')

        # Make sure the services are available
        while not self.client_1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('<service_1_name> is not available, waiting...')
        while not self.client_2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('<service_2_name> is not available, waiting...')

        self.get_logger().info('All services available, proceeding with execution')

    def call_service_1(self, request):
        """
        Call the first service and handle its response.
        """
        future = self.client_1.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error(f'Error calling service: {e}')

        self.get_logger().info(f'Received response from ChangeExposure: {response.success}')

        return response


    def call_service_2(self, request):
        """
        Call the second service and handle its response.
        """
        future = self.client_2.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.done():
            try:
                response = future.result()
            except Exception as e:
                self.get_logger().error(f'Error calling service: {e}')

        self.get_logger().info('Received response from TakePicture:')

        return response


    def take_picture(self, exposure_value=0):
        request = ChangeExposure.Request()
        request.exposure_value = exposure_value
        response = self.call_service_1(request)

        pic_request = TakePicture.Request()
        pic_request.request = True
        pic_response = self.call_service_2(pic_request)

        # Convert to np.matrix for use with openCV
        image = self.bridge.imgmsg_to_cv2(pic_response.image)

        return image

    def take_3_pictures(self, exposure_values=None):
        if exposure_values is None:
            exposure_values = [0, 0, 0]

        img1 = self.take_picture(exposure_values[0])
        img2 = self.take_picture(exposure_values[1])
        img3 = self.take_picture(exposure_values[2])

        images = [img1, img2, img3]

        return images

    def process_hdr(self, exposure_values=None):
        if exposure_values is None:
            exposure_values = [0, 0, 0]






def main():
    rclpy.init()
    try:
        node = MyServiceClientNode()
        response = node.take_3_pictures(exposure_values=[6, 1000, 2000])
        for i in range(len(response)):
            cv2.imwrite(f'//home//mark//watney_ws//pictures//image_{i}.jpg', response[i])
            node.get_logger().info(f'//home//mark//watney_ws//pictures//Save image_{i}.jpg:')

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()