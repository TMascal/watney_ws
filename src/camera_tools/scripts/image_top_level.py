#!/home/mark/ros2-humble-env/bin/python3
# File Meamt to Connect ROS Services to process, and store camera data
# Timothy Mascal
# Created 3/14/25

import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from sensor_msgs.msg import Image

import cv2
from cv_bridge import CvBridge
import numpy as np

from camera_tools_interfaces.srv import CalcClarity, VisionSystemCall

import datetime
import json

class VisionSystem(Node):
    def __init__(self):
        super().__init__('vision_controller')

        # Define ths node as a Servie
        self.service_ = self.create_service(VisionSystemCall, 'vision_controller', self.process_video)

        # Subscription to the raw camera data
        self.video_subscription = self.create_subscription(Image, 'image_raw', self.video_callback, 10)
        self.frame = None
        self.track = 0

        # Service Node for Clarity
        self.clarity_service = self.create_client(CalcClarity, '/analysis/calc_clarity')
        while not self.clarity_service.wait_for_service(timeout_sec=1.0):
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

        while True:
            # Objective 1: Take a normal picture with auto aperature adjustment
            # Must Pass Clarity -> Bightness -> Save to Raw Data Folder
            image = self.frame

            # Check if there is data
            if image is None:
                continue

            # Run Through Clarity Service, and record into
            clarity_score = self.RequestClarity(image)
            self.get_logger().log(f"Clarity Score is received {clarity_score}")


            if clarity_score < 1: # Needs to be calculated from experimental data
                self.get_logger().warn('Clarity Score is below Threshold, taking another picture')
                continue

            # If Pass, Run Through Brightness Service
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            brightness = np.mean(gray)

            if brightness < 185:
                self.get_logger().warn('Brightness is below Threshold, taking another picture')
                continue

            return


        # Save Photo and Test Results into Text File
        save_path = "~/home/mark/Pictures/" + self.current_time + "/"
        img_name = "basic_image_" + self.current_time + ".jpg"
        cv2.imwrite(save_path  + img_name, image)
        import json

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

        # Objective 2: Take a Color Accurate Photo
            # Need to Implement Hardware to do this autonomously

        # Objective 3: Take an HDR Photo

        # Objective 4: Take a Color Accurate HDR Photo

        # Return

    def RequestClarity(self, image):
        data = self.bridge.cv2_to_imgmsg(image, 'bgr8')  # Convert OpenCV image to ROS image message
        request = CalcClarity.Request()
        request.image = data

        self.get_logger().info('Calling clarity service asynchronously...')
        future = self.clarity_service.call_async(request)  # Make asynchronous call

        # Wait for the response using spin_until_future_complete
        rclpy.spin_until_future_complete(self, future)


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
    node.process_video()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
