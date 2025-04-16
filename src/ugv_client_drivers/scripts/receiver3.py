#!/home/mark/ros2-humble-env/bin/python3
import sys
import gi

gi.require_version('Gst', '1.0')
from gi.repository import Gst

def main():
    # Initialize GStreamer.
    Gst.init(None)

    # Build a pipeline that:
    # 1. Listens on UDP port 5000 for incoming RTP packets.
    # 2. Uses caps to tell GStreamer what kind of stream to expect.
    # 3. Depayloads the RTP packets into a raw H.264 stream.
    # 4. Parses the H.264 stream.
    # 5. Decodes the H.264 video.
    # 6. Converts the video to a displayable format.
    # 7. Displays the video in a window using autovideosink.
    pipeline_str = (
        'udpsrc port=5004 caps="application/x-rtp, media=video, clock-rate=90000, '
        'encoding-name=H264, payload=96" ! rtph264depay ! h264parse ! avdec_h264 ! '
        'videoconvert ! autovideosink'
    )

    print("Creating receiver pipeline...")
    pipeline = Gst.parse_launch(pipeline_str)
    if not pipeline:
        print("Failed to create receiver pipeline.")
        sys.exit(1)

    # Set the pipeline to PLAYING state to begin receiving and displaying the stream.
    ret = pipeline.set_state(Gst.State.PLAYING)
    if ret == Gst.StateChangeReturn.FAILURE:
        print("Unable to set receiver pipeline to PLAYING state.")
        sys.exit(1)

    print("Receiver pipeline is now PLAYING. Waiting for video... Press Ctrl+C to exit.")

    try:
        # Wait indefinitely until an error or EOS message is received.
        bus = pipeline.get_bus()
        msg = bus.timed_pop_filtered(Gst.CLOCK_TIME_NONE, Gst.MessageType.EOS | Gst.MessageType.ERROR)
        if msg:
            if msg.type == Gst.MessageType.ERROR:
                err, debug = msg.parse_error()
                print(f"Error: {err}, {debug}")
    except KeyboardInterrupt:
        print("Receiver interrupted by user.")
    finally:
        print("Stopping receiver pipeline...")
        pipeline.set_state(Gst.State.NULL)
        print("Receiver pipeline stopped.")

if __name__ == '__main__':
    main()
