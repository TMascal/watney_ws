#!/home/mark/ros2-humble-env/bin/python3
from urllib import response

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import numpy as np

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
            exposure_values = [100, 1000, 2000]
            exposure_times = np.array([0.01, 0.1, 0.2], dtype=np.float32)  # Converted from 100ms, 1000ms, and 2000ms

        # Step 1: Take 3 images with specified exposure values
        images = self.take_3_pictures(exposure_values=exposure_values)

        # Verify images were captured correctly
        if any(img is None for img in images):
            self.get_logger().error("Error: One or more images could not be captured.")
            return None

            # Check if any image failed to load
        if any(image is None for image in images):
            print("Error: Unable to read one or more images. Please check the image files.")
            return

        # Ensure the number of exposure times matches the number of images
        if len(images) != len(exposure_times):
            print("Error: The number of exposure times does not match the number of input images.")
            return

        # Step 3: Convert exposure values to np.array for HDR processing
        try:
            # Merge images into an HDR image
            merge_debevec = cv2.createMergeDebevec()
            hdr_image = merge_debevec.process(images, times=exposure_times)
            self.get_logger().info("HDR image successfully created.")
        except Exception as e:
            self.get_logger().error(f"Error during HDR merging: {e}")
            return None

        # Step 4: Tone mapping to convert HDR to LDR for display
        try:
            # Reinhard tone mapping (more natural result)
            tonemap_reinhard = cv2.createTonemapReinhard(gamma=1.5, intensity=0, light_adapt=0, color_adapt=1.0)
            ldr_image = tonemap_reinhard.process(hdr_image)

            # Normalize to 8-bit range for saving and display
            ldr_image = cv2.normalize(ldr_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC3)

            self.get_logger().info("Tone mapping successfully applied.")
            return ldr_image
        except Exception as e:
            self.get_logger().error(f"Error during tone mapping: {e}")
            return None

def main():
    rclpy.init()
    try:
        node = MyServiceClientNode()
        response = node.process_hdr()
        cv2.imwrite(f'//home//mark//watney_ws//pictures//image.jpg', response)
        node.get_logger().info(f'//home//mark//watney_ws//pictures//Save.jpg:')

        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()