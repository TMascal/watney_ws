#!/usr/bin/env python3
import sys
import gi
import time as t
import subprocess
import gc

gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gst, GLib, GObject

def set_camera_controls():
    """
    Set initial camera controls: switch to manual mode and set exposure to 100.
    """
    try:
        print("Setting initial camera controls: manual mode and exposure=100.")
        subprocess.check_call([
            "v4l2-ctl",
            "--device=/dev/video0",
            "--set-ctrl=auto_exposure=1",
            "--set-ctrl=exposure_time_absolute=100"
        ])
        print("Initial camera controls set successfully.")
    except subprocess.CalledProcessError as e:
        print("Error setting initial camera controls:", e)

def list_element_properties(element):
    """
    List all properties available on the given GStreamer element.
    """
    print("Listing properties for element '%s':" % element.get_name())
    for prop in element.list_properties():
        print(f"  {prop.name} (type: {prop.value_type.name})")

def list_extra_controls(v4l2src):
    """
    Retrieve and list all fields available in the extra-controls GstStructure.
    """
    try:
        extra = v4l2src.get_property("extra-controls")
        if extra:
            print("Current extra-controls:")
            # Iterate over all fields in the GstStructure.
            for key in extra.keys():
                print(f"  {key} : {extra.get_value(key)}")
        else:
            print("No extra-controls property is set on the element.")
    except Exception as e:
        print("Failed to get extra-controls:", e)

def update_exposure_extra_controls(v4l2src):
    """
    Attempt to update the exposure via the extra-controls property.

    This builds a new GstStructure with the desired extra controls.
    """
    try:
        # Create a new GstStructure for extra controls.
        # The structure name "v4l2-extra-controls" is arbitrary and may vary;
        # some implementations may require specific structure names.
        controls = Gst.Structure.new_empty("v4l2-extra-controls")
        # Set the "exposure_time_absolute" control to 1000.
        controls.set_value("exposure_time_absolute", 1000)
        # Assign the new structure to the "extra-controls" property.
        v4l2src.set_property("extra-controls", controls)
        print("Extra-controls updated: exposure_time_absolute set to 1000")
    except Exception as e:
        print("Failed to update exposure via extra-controls:", e)

def main():
    # Set initial camera controls.
    set_camera_controls()

    # Initialize GStreamer.
    Gst.init(None)

    # Build the pipeline string.
    # Give the v4l2src element an explicit name ("myv4l2src") so we can reference it.
    pipeline_str = (
        'v4l2src name=myv4l2src device=/dev/video0 ! '
        'capsfilter caps="image/jpeg, width=1280, height=720, framerate=30/1" ! '
        'jpegdec ! videoconvert ! '
        'x264enc bitrate=1500 speed-preset=superfast tune=zerolatency ! '
        'h264parse ! rtph264pay config-interval=1 pt=96 ! '
        'udpsink host=10.33.175.6 port=5000'
    )

    print("Creating sender pipeline (attempting extra-controls update with extra-controls listing)...")
    pipeline = Gst.parse_launch(pipeline_str)
    if not pipeline:
        print("Failed to create sender pipeline.")
        sys.exit(1)

    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("Unable to set sender pipeline to PLAYING state.")
        sys.exit(1)

    print("Sender pipeline is now PLAYING. Streaming video...")
    print("After 5 seconds, the script will list properties and extra-controls for v4l2src,")
    print("and attempt to update exposure (exposure_time_absolute) to 1000 via extra-controls.")

    # Schedule a one-time timer after 5 seconds to list properties/extra-controls and perform update.
    def update_and_list_properties():
        # Retrieve the v4l2src element by its name.
        v4l2src = pipeline.get_by_name("myv4l2src")
        if v4l2src:
            list_element_properties(v4l2src)
            list_extra_controls(v4l2src)
            update_exposure_extra_controls(v4l2src)
            print("After attempting update, extra-controls are now:")
            list_extra_controls(v4l2src)
        else:
            print("v4l2src element 'myv4l2src' not found.")
        # Return False to run this timeout only once.
        return False

    GLib.timeout_add_seconds(5, update_and_list_properties)

    # Create a GLib MainLoop to manage GStreamer messages.
    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def bus_call(bus, message, loop):
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print("Error:", err, debug)
            loop.quit()
        elif message.type == Gst.MessageType.EOS:
            print("End-of-stream")
            loop.quit()
        return True

    bus.connect("message", bus_call, loop)

    try:
        loop.run()
    except KeyboardInterrupt:
        print("Sender interrupted by user.")
        loop.quit()
    finally:
        print("Stopping sender pipeline...")
        pipeline.set_state(Gst.State.NULL)
        del pipeline
        gc.collect()
        t.sleep(3)
        print("Sender pipeline stopped.")

if __name__ == '__main__':
    main()
