#!/usr/bin/env python3
import sys
import gi
import time as t
import gc

gi.require_version('Gst', '1.0')
gi.require_version('GLib', '2.0')
from gi.repository import Gst, GLib

def main():
    # Initialize GStreamer.
    Gst.init(None)

    # Build the pipeline with a capsfilter element.
    pipeline_str = (
        'v4l2src device=/dev/video0 ! '
        'capsfilter caps="image/jpeg, width=1280, height=720, framerate=30/1" ! '
        'jpegdec ! videoconvert ! '
        'x264enc bitrate=1500 speed-preset=superfast tune=zerolatency ! '
        'h264parse ! rtph264pay config-interval=1 pt=96 ! '
        'udpsink host=10.33.175.6 port=5000'
    )

    print("Creating sender pipeline...")
    pipeline = Gst.parse_launch(pipeline_str)
    if not pipeline:
        print("Failed to create sender pipeline.")
        sys.exit(1)

    # Start streaming.
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("Unable to set sender pipeline to PLAYING state.")
        sys.exit(1)

    print("Sender pipeline is now PLAYING. Streaming video... Press Ctrl+C to exit.")

    # Create a GLib MainLoop to manage GStreamer messages and graceful exit.
    loop = GLib.MainLoop()
    bus = pipeline.get_bus()
    bus.add_signal_watch()

    def bus_call(bus, message, loop):
        if message.type == Gst.MessageType.ERROR:
            err, debug = message.parse_error()
            print(f"Error: {err}, {debug}")
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
        t.sleep(3)  # Pause briefly to allow for cleanup
        print("Sender pipeline stopped.")

if __name__ == '__main__':
    main()
