#!/usr/bin/env python3
# File is the UGV Unified Driver
# Timothy Mascal
# Created 3/14/25

import gi
gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gst, GLib, GObject
import sys
import subprocess

import rclpy
from rclpy.node import Node
from rclpy.callback_groups import ReentrantCallbackGroup

from camera_tools_interfaces.srv import ChangeExposure

class UGVDriver(Node):
    def __init__(self):
        super().__init__('ugv_driver')

        # Declare parameters with defaults
        self.declare_parameter('video_device_index', "/dev/video0")
        self.declare_parameter('pipeline_port', 5000)
        self.declare_parameter('ip_address', "192.168.0.105")
        self.declare_parameter('w', 1280)
        self.declare_parameter("h", 720)
        self.declare_parameter("hz", 30)

        # Retrieve parameter values
        self.video_device_index = self.get_parameter('video_device_index').value
        self.pipeline_port = self.get_parameter('pipeline_port').value
        self.ip_address = self.get_parameter('ip_address').value
        self.width = self.get_parameter("w").value
        self.height = self.get_parameter("h").value
        self.refresh = self.get_parameter("hz").value

        self.get_logger().info(f"Using video device: {self.video_device_index}")
        self.get_logger().info(f"Using pipeline port: {self.pipeline_port}")
        self.get_logger().info(f"Using ip address: {self.ip_address}")

        # define callback groups
        group1 = ReentrantCallbackGroup()

        # Define ths node as a Servie
        self.service_ = self.create_service(ChangeExposure, 'change_exposure', self.change_exposure, callback_group=group1)

        # Start GStreamer Pipeline
        self.GStreamerSetup()

    def change_exposure(self, request, response):
        """
        Change the camera exposure using the update_exposure function.

        :param request: An object that has an attribute `exposure_value`
        :param response: An object that has an attribute `success`
        """
        # Extract the new exposure value from the request.
        new_exposure_value = request.exposure_value
        response.success = False  # Default to False

        self.get_logger().info(f"The Exposure Value Received is {new_exposure_value}")

        if new_exposure_value == 0:
            # If the exposure value is zero, we interpret that as a request to set
            # the camera to automatic exposure mode. In this case, we call update_exposure
            # with an auto_exposure value of 3 (commonly indicating auto mode) and set
            # the exposure_value to 0 (or any value that your driver ignores in auto mode).
            self.update_exposure(0, auto_exposure_value=3)
            response.success = True
            self.get_logger().info("Command executed successfully, exposure set to automatic")

        else:
            # For any nonzero exposure value, set the camera to manual mode (auto_exposure=1)
            # and update the exposure_time_absolute setting to new_exposure_value.
            self.update_exposure(new_exposure_value, auto_exposure_value=1)
            response.success = True
            self.get_logger().info(f"Command executed successfully, exposure set to: {new_exposure_value}")


    def GStreamerSetup(self):
        # Set initial camera controls.
        self.set_camera_controls()

        # Initialize GStreamer.
        Gst.init(None)

        # Build the pipeline string.
        # Give the v4l2src element an explicit name ("myv4l2src") so we can reference it.
        pipeline_str = (
            f'v4l2src name=myv4l2src device={self.video_device_index} ! '
            f'capsfilter caps="image/jpeg, width={self.width}, height={self.height}, framerate={self.refresh}/1" ! '
            'jpegdec ! videoconvert ! '
            'x264enc bitrate=1500 speed-preset=superfast tune=zerolatency ! '
            'h264parse ! rtph264pay config-interval=1 pt=96 ! '
            f'udpsink host={self.ip_address} port={self.pipeline_port}'
            # Eagle Net, 10.33.175.6, TP-LINK: 192.168.0.105

        )
        self.get_logger().info(pipeline_str)
        self.get_logger().info("Creating sender pipeline (attempting extra-controls update with extra-controls listing)...")
        self.pipeline = Gst.parse_launch(pipeline_str)
        if not self.pipeline:
            self.get_logger().error("Failed to create sender pipeline.")
            sys.exit(1)

        ret = self.pipeline.set_state(Gst.State.PLAYING)
        if ret == Gst.StateChangeReturn.FAILURE:
            self.get_logger().error("Unable to set sender pipeline to PLAYING state.")
            sys.exit(1)

        self.get_logger().info("Sender pipeline is now PLAYING. Streaming video...")
        return 0

    def set_camera_controls(self):
        """
        Set initial camera controls: switch to manual mode and set exposure to 100.
        """
        try:
            self.get_logger().info("Setting initial camera controls: manual mode and exposure=100.")
            subprocess.check_call([
                "v4l2-ctl",
                f"--device={self.video_device_index}",
                "--set-ctrl=auto_exposure=3"
            ])
            self.get_logger().info("Initial camera controls set successfully.")
        except subprocess.CalledProcessError as e:
            self.get_logger().error("Error setting initial camera controls:")


    def update_exposure(self, exposure_value, auto_exposure_value):
        """
        Attempt to update the exposure via the extra-controls property.

        This builds a new GstStructure with the desired extra controls.
        """

        # Retrieve the v4l2src element by its name.
        v4l2src = self.pipeline.get_by_name("myv4l2src")

        if not v4l2src:
            print("v4l2src element 'myv4l2src' not found.")
            return

        try:
            # Create a new GstStructure for extra controls.
            controls = Gst.Structure.new_empty("v4l2-extra-controls")

            # Set the "auto_exposure" control
            controls.set_value("auto_exposure", auto_exposure_value)

            # Set the "exposure_time_absolute" control
            controls.set_value("exposure_time_absolute", exposure_value)

            # Assign the new structure to the "extra-controls" property.
            v4l2src.set_property("extra-controls", controls)
            self.get_logger().info(f"Extra-controls updated: auto_exposure set to {auto_exposure_value}")
            self.get_logger().info(f"Extra-controls updated: exposure_time_absolute set to {exposure_value}")
        except Exception as e:
            self.get_logger().error("Failed to update exposure via extra-controls:", e)

def main(args=None):
    rclpy.init(args=args)
    node = UGVDriver()
    rclpy.spin_once(node)
    executor = rclpy.executors.MultiThreadedExecutor()
    executor.add_node(node)
    executor.spin()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
