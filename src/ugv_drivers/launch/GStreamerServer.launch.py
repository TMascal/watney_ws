#!/usr/bin/env python3
import glob
import subprocess
from launch import LaunchDescription
from launch.actions import ExecuteProcess, OpaqueFunction
from launch_ros.actions import Node  # Import Node action for ROS nodes

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

    # Example: Build a GStreamer pipeline command for a camera (e.g., front_cam).
    if "front_cam" in devices:
        command = (
            f'gst-launch-1.0 -v v4l2src device={devices["front_cam"]} ! '
            f'"image/jpeg, width=1280, height=720, framerate=30/1" ! '
            'jpegdec ! '
            'videoconvert ! '
            'x264enc bitrate=1500 speed-preset=superfast tune=zerolatency ! '
            'h264parse ! '
            'rtph264pay config-interval=1 pt=96 ! '
            'udpsink host=192.168.0.105 port=5000'
        )
        actions.append(
            ExecuteProcess(
                cmd=['bash', '-c', command],
                output='screen'
            )
        )
    else:
        print("front_cam device not found in mapping.")

    # Now, add the node equivalent to the XML launch file for change_exposure_server.
    actions.append(
        Node(
            package="ugv_drivers",
            executable="change_exposure_server",
            name="change_exposure",
            namespace="top_cam",
            output="screen"
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
