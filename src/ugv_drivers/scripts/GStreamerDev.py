#!/usr/bin/env python3
import gi
import time

gi.require_version('Gst', '1.0')
from gi.repository import Gst, GLib

def query_extra_controls(element):
    """
    Print the extra-controls from the given element.
    If the control structure is available, iterate through and print each key and its value.
    """
    controls = element.get_property("extra-controls")
    if controls:
        print("Extra Controls:")
        for key in controls.keys():
            value = controls.get_value(key)
            print(f"  {key}: {value}")
    else:
        print("No extra-controls available on this device.")

def modify_extra_controls(element, auto_exposure_val, exposure_time_val=None):
    """
    Modify the extra-controls of the given element.
      - auto_exposure_val: Desired value for auto_exposure (e.g. 1 to set manual mode).
      - exposure_time_val: If provided, the desired value for exposure_time_absolute.
    This function copies the current extra-controls structure, sets the new values,
    and applies the new structure to the element.
    """
    current = element.get_property("extra-controls")
    if not current:
        print("No extra-controls available; cannot modify exposure.")
        return

    new_controls = current.copy()

    print(f"Setting auto_exposure to {auto_exposure_val} ...")
    new_controls.set_value("auto_exposure", auto_exposure_val)

    if exposure_time_val is not None:
        print(f"Setting exposure_time_absolute to {exposure_time_val} ...")
        new_controls.set_value("exposure_time_absolute", exposure_time_val)

    element.set_property("extra-controls", new_controls)
    print("Modified extra-controls have been applied.")

def main():
    # Initialize GStreamer.
    Gst.init(None)

    # Construct the sender pipeline:
    # - v4l2src: Capture from /dev/video0 with MJPEG format.
    # - jpegdec: Decode the MJPEG frames.
    # - videoconvert: Convert formats if necessary.
    # - x264enc: Encode frames to H.264.
    # - h264parse: Parse the H.264 stream.
    # - rtph264pay: Packetize it for RTP.
    # - udpsink: Transmit the RTP stream over UDP.
    pipeline_str = (
        'v4l2src name=cam_src device=/dev/video0 ! '
        'image/jpeg, width=1280, height=720, framerate=30/1 ! '
        'jpegdec ! videoconvert ! '
        'x264enc bitrate=1500 speed-preset=superfast tune=zerolatency ! '
        'h264parse ! rtph264pay config-interval=1 pt=96 ! '
        'udpsink host=10.33.175.6 port=5000'
    )

    pipeline = Gst.parse_launch(pipeline_str)
    if not pipeline:
        print("Failed to create pipeline!")
        return

    # Set the pipeline to PLAYING (this starts capturing and streaming).
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret != Gst.StateChangeReturn.SUCCESS and ret != Gst.StateChangeReturn.ASYNC:
        print("Unable to set pipeline to PLAYING state.")
        return

    print("Pipeline is now PLAYING. Streaming for 5 seconds...")
    time.sleep(5)

    # Transition to PAUSED before modifying camera controls.
    print("Pausing pipeline to modify exposure controls...")
    pipeline.set_state(Gst.State.PAUSED)
    time.sleep(2)  # Allow time for state change to settle.

    # Retrieve the v4l2src element named 'cam_src'.
    cam_src = pipeline.get_by_name("cam_src")
    if not cam_src:
        print("Could not find element 'cam_src'.")
    else:
        print("\nExtra-controls BEFORE modification:")
        query_extra_controls(cam_src)

        # Modify the controls (for example, set auto_exposure to 1 (manual)
        # and set exposure_time_absolute to 100).
        modify_extra_controls(cam_src, auto_exposure_val=1, exposure_time_val=100)

        print("\nExtra-controls AFTER modification:")
        query_extra_controls(cam_src)

    # Resume streaming by setting the pipeline back to PLAYING.
    print("Resuming pipeline to PLAYING state...")
    pipeline.set_state(Gst.State.PLAYING)

    # Continue streaming for 10 seconds so you can observe changes.
    time.sleep(10)

    print("Stopping pipeline...")
    pipeline.set_state(Gst.State.NULL)
    print("Pipeline stopped.")

if __name__ == '__main__':
    main()
