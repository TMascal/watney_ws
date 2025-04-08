#!/home/mark/ros2-humble-env/bin/python3
# File Meamt to Connect ROS Services to process, and store camera data
# Timothy Mascal
# Created 3/14/25

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np
from threading import Event

from camera_tools_interfaces.srv import CalcClarity, VisionSystemCall, HDRService

import datetime
import json
import os

class VisionSystem(Node):
    def __init__(self):
        super().__init__('vision_controller')

        # define callback groups
        group1 = ReentrantCallbackGroup()
        group2 = ReentrantCallbackGroup()
        group3 = ReentrantCallbackGroup()
        group4 = ReentrantCallbackGroup()

        # Define ths node as a Servie
        self.service_ = self.create_service(VisionSystemCall, 'vision_controller', self.process_video, callback_group=group1)

        # Subscription to the raw camera data
        self.video_subscription = self.create_subscription(Image, 'image_raw', self.video_callback, 10, callback_group=group2)
        self.frame = None

        # Service Node for Clarity
        self.clarity_service = self.create_client(CalcClarity, '/analysis/calc_clarity', callback_group=group3)
        while not self.clarity_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Service Node for HDR
        self.hdr_service = self.create_client(HDRService, 'hdr_photo', callback_group=group4)
        while not self.hdr_service.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Service not available, waiting...')

        # Initialize OpenCV Bridge
        self.bridge = CvBridge()

        # Data for Files
        self.current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')


        self.get_logger().info('Vision Controller Node Initialized')

    def video_callback(self, msg):
        self.frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')

    def process_video(self, request, response):
        # Get Data
        self.current_time = datetime.datetime.now().strftime('%Y-%m-%d_%H-%M-%S')
        aruco_id = request.id
        self.get_logger().info(f'Received request for ID: {aruco_id}')

        while True:
            self.get_logger().info('Processing Video')
            # Objective 1: Take a normal picture with auto aperature adjustment
            # Must Pass Clarity -> Bightness -> Save to Raw Data Folder
            image = self.frame

            # Check if there is data
            if image is None:
                continue

            # Run Through Clarity Service, and record into
            clarity_score = self.RequestClarity(image)
            self.get_logger().info(f"Clarity Score is received: {clarity_score}")


            if clarity_score < 0.5: # Needs to be calculated from experimental data
                self.get_logger().warn('Clarity Score is below Threshold, taking another picture')
                continue

            # If Pass, Run Through Brightness Service
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            brightness = np.mean(gray)
            self.get_logger().info(f"Brightness is received: {brightness}")

            if brightness < 100:
                self.get_logger().warn('Brightness is below Threshold, taking another picture')
                continue

            self.get_logger().info('End Loop, saving data')
            break


        # Save Photo and Test Results into Text File
        save_path = os.path.expanduser("~/Pictures/") + self.current_time + "/"
        os.makedirs(save_path, exist_ok=True)  # Create the directory if it doesn't exist
        img_name = "basic_image_" + self.current_time + ".jpg"
        cv2.imwrite(save_path  + img_name, image)
        self.get_logger().info(f"Image Saved to {save_path}")

        # Load existing data or create an empty list
        try:
            with open(save_path + "metadata.json", "r") as file:
                metadata = json.load(file)
        except FileNotFoundError:
            metadata = []

        # Add new photo data
        metadata.append({
            "image_name": img_name,
            "timestamp": self.current_time,
            "clarity_score": clarity_score,
            "brightness": brightness,
        })

        # Write back the updated data to JSON
        with open(save_path + "metadata.json", "w") as file:
            json.dump(metadata, file, indent=4)

        self.get_logger().info(f"Metadata Saved to {save_path}")

        # Objective 2: Take a Color Accurate Photo
            # Need to Implement Hardware to do this autonomously

        # Objective 3: Take an HDR Photo
        while True:
            self.get_logger().info('Starting HDR Photo Process')
            # Must Pass Clarity -> Bightness -> Save to Raw Data Folder
            HDR_data = self.CallHDR()
            image_hdr = self.bridge.imgmsg_to_cv2(HDR_data.image_hdr)
            image_low = self.bridge.imgmsg_to_cv2(HDR_data.image_low)
            image_mid = self.bridge.imgmsg_to_cv2(HDR_data.image_mid)
            image_high = self.bridge.imgmsg_to_cv2(HDR_data.image_high)

            # Check if there is data
            if image_hdr is None:
                continue

            # Run Through Clarity Service, and record into
            clarity_score = self.RequestClarity(image_hdr)
            self.get_logger().info(f"Clarity Score is received: {clarity_score}")


            if clarity_score < 0.5: # Needs to be calculated from experimental data
                self.get_logger().warn('Clarity Score is below Threshold, taking another picture')
                continue

            # If Pass, Run Through Brightness Service
            gray = cv2.cvtColor(image_hdr, cv2.COLOR_BGR2GRAY)
            brightness = np.mean(gray)
            self.get_logger().info(f"Brightness is received: {brightness}")

            if brightness < 100:
                self.get_logger().warn('Brightness is below Threshold, taking another picture')
                continue

            self.get_logger().info('End Loop, saving data')
            break

        # Save HDR Data
        # Save Photo and Test Results into Text File
        img_name = "hdr_image_" + self.current_time + ".jpg"
        cv2.imwrite(save_path  + img_name, image_hdr)
        self.get_logger().info(f"Image Saved to {save_path}")

        # Load existing data or create an empty list
        try:
            with open(save_path + "metadata.json", "r") as file:
                metadata = json.load(file)
        except FileNotFoundError:
            metadata = []

        # Add new photo data
        metadata.append({
            "image_name": img_name,
            "timestamp": self.current_time,
            "clarity_score": clarity_score,
            "brightness": brightness,
        })

        # Write back the updated data to JSON
        with open(save_path + "metadata.json", "w") as file:
            json.dump(metadata, file, indent=4)

        self.get_logger().info(f"Metadata Saved to {save_path}")

        # List of images and their corresponding level names
        source_images = [image_low, image_mid, image_high]
        levels = ["low", "mid", "high"]
        hdr_save_path = os.path.join(save_path, "HDR_Source")
        os.makedirs(hdr_save_path, exist_ok=True)  # Create the directory if it doesn't exist
        # Loop over each image and its associated level name
        for img, level in zip(source_images, levels):
            # Construct the image name with the level appended before the file extension
            img_name = f"hdr_image_{self.current_time}_{level}.jpg"
            # Build the full path for saving the image; using os.path.join for cross-platform compatibility
            full_path = os.path.join(hdr_save_path, img_name)

            # Save the image using OpenCV
            cv2.imwrite(full_path, img)

            # Log the saved image path
            self.get_logger().info(f"Image saved to {full_path}")


        # Objective 4: Take a Color Accurate HDR Photo

        # Return file path
        response.file_path = save_path

        return response

    def CallHDR(self):
        request = HDRService.Request()
        request.request = True

        self.get_logger().info('Calling HDR service asynchronously...')
        future = self.hdr_service.call_async(request)  # Make asynchronous call

        # Use a threading Event to wait for the future without blocking the executor's callbacks
        done_evt = Event()
        future.add_done_callback(lambda fut: done_evt.set())

        # Wait for up to 5 seconds for the future to complete
        if not done_evt.wait(timeout=5.0):
            self.get_logger().error("Timeout waiting for HDR service response")
            return None

        if future.done():
            try:
                response = future.result()  # Get the service response
                if response is not None:
                    self.get_logger().info('HDR Photos received!')
                    return response
                else:
                    self.get_logger().warn('No response received from clarity service.')
                    return None
            except Exception as e:
                self.get_logger().error(f'Exception during clarity service call: {str(e)}')
                return None



    def RequestClarity(self, image):
        data = self.bridge.cv2_to_imgmsg(image, 'bgr8')  # Convert OpenCV image to ROS image message
        request = CalcClarity.Request()
        request.image = data

        self.get_logger().info('Calling clarity service asynchronously...')
        future = self.clarity_service.call_async(request)  # Make asynchronous call

        # Use a threading Event to wait for the future without blocking the executor's callbacks
        done_evt = Event()
        future.add_done_callback(lambda fut: done_evt.set())

        # Wait for up to 5 seconds for the future to complete
        if not done_evt.wait(timeout=5.0):
            self.get_logger().error("Timeout waiting for clarity service response")
            return None

        if future.done():
            try:
                response = future.result()  # Get the service response
                if response is not None:
                    self.get_logger().info(f'Clarity score received: {response.clarity_value}')
                    return response.clarity_value
                else:
                    self.get_logger().warn('No response received from clarity service.')
                    return None
            except Exception as e:
                self.get_logger().error(f'Exception during clarity service call: {str(e)}')
                return None


def main(args=None):
    rclpy.init(args=args)
    node = VisionSystem()
    rclpy.spin_once(node)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
