#!/home/mark/ros2-humble-env/bin/python3
from urllib import response

import cv2
from cv_bridge import CvBridge
import rclpy
from rclpy.node import Node
import numpy as np
from rclpy.callback_groups import ReentrantCallbackGroup
from sensor_msgs.msg import Image
from threading import Event

from camera_tools_interfaces.srv import ChangeExposure, TakePicture

class MyServiceClientNode(Node):
    def __init__(self):
        super().__init__('hdr_photo_node')

        # define callback groups
        group1 = ReentrantCallbackGroup()
        group2 = ReentrantCallbackGroup()
        group3 = ReentrantCallbackGroup()

        # Create CV2 Bridge Object
        self.bridge = CvBridge()

        # Create a threading event to signal when a new frame is available.
        self.frame_updated_event = Event()
        # Counter to track the number of frames received.
        self.frame_counter = 0

        # Create the service clients
        self.exposure_srv = self.create_client(ChangeExposure, 'change_exposure', callback_group=group1)

        # Link Camera Topic
        # Subscription to the raw camera data
        self.video_subscription = self.create_subscription(Image, 'image_raw', self.video_callback, 10, callback_group=group2)
        self.frame = None

        # Create Service Server
        self.service_ = self.create_service(TakePicture, 'hdr_photo', self.process_request, callback_group=group3)

        # Make sure the services are available
        while not self.exposure_srv.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('/change_exposure is not available, waiting...')
        self.get_logger().info('All services available, proceeding with execution')

    def video_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        # Increment the counter.
        self.frame_counter += 1
        # Signal that a new frame has been received
        self.frame_updated_event.set()
        return

    def process_request(self, request, response):
        bridge = CvBridge()

        # Create a 512x512 white image with 3 channels (BGR)
        white_image = np.full((5, 5, 3), 255, dtype=np.uint8)

        self.get_logger().info(f"Created image of type: {type(white_image)} with shape: {white_image.shape}")

        # Convert the OpenCV image (NumPy array) to a ROS Image message.
        ros_image_msg = bridge.cv2_to_imgmsg(white_image, encoding="bgr8")

        # Package the ROS Image message into the response.
        response.image = ros_image_msg

        self.take_3_pictures([100, 1000, 2000])

        return response

    def set_camera_exposure(self, exposure_value):
        """
        Call the service to change the exposure of the camera
        """
        request = ChangeExposure.Request()
        request.exposure_value = exposure_value

        future = self.exposure_srv.call_async(request)

        # Create an Event to wait for the future to complete without blocking the executor.
        done_evt = Event()
        future.add_done_callback(lambda fut: done_evt.set())

        # Wait for up to 5 seconds for the future to complete.
        if not done_evt.wait(timeout=5.0):
            self.get_logger().error("Timeout waiting for service response")
            return None

        try:
            response = future.result()
        except Exception as e:
            self.get_logger().error(f'Error calling service: {e}')
            return None

        self.get_logger().info(f'Received response from ChangeExposure: {response.success}')

        return response

    def take_3_pictures(self, exposure_values=(0, 0, 0), frames_to_wait=5):

        images = []
        for exposure in exposure_values:
            # Record the current frame counter.
            baseline = self.frame_counter

            # Clear the event so we wait for new frame events.
            self.frame_updated_event.clear()

            # Set the exposure.
            self.set_camera_exposure(exposure)

            # Wait until 'frames_to_wait' new frames have been received.
            target_count = baseline + frames_to_wait
            while self.frame_counter < target_count:
                # Wait with a timeout to prevent infinite blocking.
                self.frame_updated_event.wait(timeout=1.0)
                self.frame_updated_event.clear()

            self.get_logger().info(f"Received {frames_to_wait} new frame events after setting exposure {exposure}")
            # Append the current frame after the required number of updates.
            images.append(self.frame)

        # Optionally save the images.
        for i, img in enumerate(images, start=1):
            if img is not None:
                filepath = f'/home/mark/watney_ws/pictures/image{i}.jpg'
                cv2.imwrite(filepath, img)
        self.get_logger().info('Saved images to /home/mark/watney_ws/pictures/')

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
    try:
        rclpy.init()
        node = MyServiceClientNode()
        rclpy.spin_once(node)
        executor = rclpy.executors.MultiThreadedExecutor()
        executor.add_node(node)
        executor.spin()

        # node.take_3_pictures([100, 1000, 2000])
        # response = node.process_hdr()
        # cv2.imwrite(f'/home/mark/watney_ws/pictures/image.jpg', response)
        # node.get_logger().info(f'//home//mark//watney_ws//pictures//Save.jpg:')


        node.destroy_node()
        rclpy.shutdown()

    except KeyboardInterrupt:
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()