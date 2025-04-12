#!/usr/bin/env python3
import launch
from launch import LaunchDescription
from launch.actions import ExecuteProcess
def generate_launch_description():
    # The complete GStreamer pipeline as a single command string.
    command = (
        'gst-launch-1.0 -v v4l2src device=/dev/video0 ! '
        '"image/jpeg, width=1280, height=720, framerate=30/1" ! '
        'jpegdec ! '
        'videoconvert ! '
        'x264enc bitrate=1500 speed-preset=superfast tune=zerolatency ! '
        'h264parse ! '
        'rtph264pay config-interval=1 pt=96 ! '
        'udpsink host=10.33.175.6 port=5000'
    )

    return LaunchDescription([
        ExecuteProcess(
            # Running the command using bash -c to properly handle the piped command string.
            cmd=['bash', '-c', command],
            output='screen'  # This ensures that the output of the process is printed to the screen.
        )
    ])
