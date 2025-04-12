#!/home/mark/ros2-humble-env/bin/python3
# Sourced from GSCam Tutorials, April 12, 2025 and modified by Tim Mascal

import cv2
import numpy as np
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

import gi
gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib
import signal  # For catching Ctrl-C

class GStreamerPublisher(Node):
    def __init__(self):
        super().__init__('gst_publisher')
        self.get_logger().info("Initializing GStreamerPublisher node...")
        self.publisher_ = self.create_publisher(Image, 'image_raw', 0)
        self.bridge = CvBridge()

        # Initialize GStreamer
        Gst.init(None)
        self.loop = GLib.MainLoop()

        # Define the GStreamer pipeline string, ending with an appsink.
        self.pipeline_str = (
            'udpsrc port=5000 caps="application/x-rtp, encoding-name=H264, payload=96" ! '
            'rtph264depay ! '
            'avdec_h264 ! '
            'videoconvert ! '
            'appsink name=sink emit-signals=true max-buffers=1 drop=true'
        )
        self.get_logger().info(f"Creating GStreamer pipeline: {self.pipeline_str}")
        self.pipeline = Gst.parse_launch(self.pipeline_str)
        self.appsink = self.pipeline.get_by_name('sink')
        if not self.appsink:
            self.get_logger().error("Could not retrieve appsink from the pipeline.")
            return

        self.appsink.connect('new-sample', self.on_new_sample)
        self.get_logger().info("Setting pipeline to PLAYING state.")
        self.pipeline.set_state(Gst.State.PLAYING)

        GLib.timeout_add_seconds(1, self.spin_once)

    def spin_once(self):
        if rclpy.ok():
            rclpy.spin_once(self, timeout_sec=0)
        return True

    def on_new_sample(self, sink):
        if not rclpy.ok():
            return Gst.FlowReturn.OK

        try:
            sample = sink.emit('pull-sample')
            if sample:
                buf = sample.get_buffer()
                caps = sample.get_caps()
                structure = caps.get_structure(0)
                width = structure.get_int("width")[1]
                height = structure.get_int("height")[1]

                result, mapinfo = buf.map(Gst.MapFlags.READ)
                if result:
                    frame = np.frombuffer(mapinfo.data, np.uint8)
                    try:
                        frame = cv2.cvtColor(frame.reshape((height * 3 // 2, width)), cv2.COLOR_YUV2BGR_I420)
                    except Exception as e:
                        self.get_logger().error(f"Error converting frame: {e}")
                    buf.unmap(mapinfo)

                    msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                    self.publisher_.publish(msg)
                    self.get_logger().debug(f"Published an image frame of size {width}x{height}.")
        except Exception as e:
            self.get_logger().error(f"Error in on_new_sample: {e}")
        return Gst.FlowReturn.OK

def main(args=None):
    rclpy.init(args=args)
    node = GStreamerPublisher()

    # Register a SIGINT (Ctrl-C) handler with GLib
    def on_sigint():
        node.get_logger().info("SIGINT received, quitting GLib main loop.")
        node.loop.quit()
        return False  # Returning False unregisters the signal source

    GLib.unix_signal_add(GLib.PRIORITY_DEFAULT, signal.SIGINT, on_sigint)

    try:
        node.get_logger().info("Running GStreamerPublisher... Press Ctrl-C to exit.")
        node.loop.run()  # This will block until node.loop.quit() is called.
    except Exception as e:
        node.get_logger().error(f"Unexpected error: {e}")
    finally:
        node.get_logger().info("Shutting down GStreamer pipeline and ROS node.")
        node.pipeline.set_state(Gst.State.NULL)
        node.destroy_node()
        rclpy.shutdown()
        print("GStreamerPublisher node has shut down.")

if __name__ == '__main__':
    main()
