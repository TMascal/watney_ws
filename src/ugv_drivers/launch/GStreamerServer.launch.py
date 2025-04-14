#!/usr/bin/env python3
import glob
import subprocess
from launch import LaunchDescription
from launch.actions import OpaqueFunction
from launch_ros.actions import Node

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
    # Define the mapping criteria for cameras.
    camera_mapping = {
        "front_cam": "1.1.2",
        "side_cam":  "1.1.3",
        "top_cam":   "1.1.4"
    }

    # Map the available video devices.
    devices = map_camera_devices(camera_mapping)
    for label, dev in devices.items():
        print(f"Mapping: {label} -> {dev}")

    if "front_cam" in devices:
        front_cam_device = devices["front_cam"]
    else:
        front_cam_device = "/dev/video0"
        print("front_cam device not found in mapping, defaulting to /dev/video0")

    # Launch the node equivalent to the XML file, passing both parameters.
    node = Node(
        package="ugv_drivers",
        executable="UGVDriver.py",  # Adjust the executable name if necessary.
        name="change_exposure",
        namespace="top_cam",
        parameters=[
            {"video_device": front_cam_device},
            {"pipeline_port": 5000},
            {"ip_address": "10.33.175.6"}
        ],
        output="screen"
    )

    return [node]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
