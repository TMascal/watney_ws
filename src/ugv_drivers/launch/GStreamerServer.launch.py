#!/usr/bin/env python3
import glob
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction

def get_id_path(device):
    """
    Returns the ID_PATH property for a given video device.
    """
    try:
        output = subprocess.check_output(
            ["udevadm", "info", "--query=property", "--name", device],
            universal_newlines=True
        )
        for line in output.splitlines():
            if line.startswith("ID_PATH="):
                return line.split("=", 1)[1].strip()
    except subprocess.CalledProcessError as e:
        print(f"Error querying {device}: {e}")
    return None

def map_camera_devices(camera_mapping):
    """
    Scans /dev/video* and returns a dictionary mapping camera labels to
    their device nodes if the device's ID_PATH contains the expected substring.

    :param camera_mapping: A dict mapping labels to expected USB path substrings.
                           For example: {"front_cam": "1-1.1.2", "side_cam": "1-1.1.3", "top_cam": "1-1.1.4"}
    :return: Dict mapping each label (if matched) to its device node (e.g. "/dev/video0").
    """
    mapped_devices = {}
    for device in glob.glob("/dev/video*"):
        id_path = get_id_path(device)
        if id_path:
            for label, expected in camera_mapping.items():
                if expected in id_path:
                    mapped_devices[label] = device
    return mapped_devices

def launch_setup(context, *args, **kwargs):
    # Define the expected mapping.
    # Update the expected substrings to match your cameras’ USB port identifiers (e.g., from udevadm info).
    camera_mapping = {
        "front_cam": "1.1.2",
        "side_cam":  "1.1.3",
        "top_cam":   "1.1.4"
    }

    devices = map_camera_devices(camera_mapping)
    for label, dev in devices.items():
        print(f"Mapping: {label} -> {dev}")

    actions = []
    # For each camera found, build a GStreamer pipeline command.
    # In this example, the same pipeline command is used for all cameras. You can customize it per camera label if needed.
    # Example command: capture MJPEG images, decode, convert,
    # encode to H.264, packetize into RTP, and send via UDP.
    command = (
        f'gst-launch-1.0 -v v4l2src device={devices["front_cam"]} ! '
        f'"image/jpeg, width=1280, height=720, framerate=30/1" ! '
        'jpegdec ! '
        'videoconvert ! '
        'x264enc bitrate=1500 speed-preset=superfast tune=zerolatency ! '
        'h264parse ! '
        'rtph264pay config-interval=1 pt=96 ! '
        'udpsink host=10.33.175.6 port=5000'
    )
    actions.append(
        ExecuteProcess(
            cmd=['bash', '-c', command],
            output='screen'
        )
    )

    if not actions:
        print("No camera devices matched the provided mapping. Check your udev attributes and mapping criteria.")
    return actions

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
