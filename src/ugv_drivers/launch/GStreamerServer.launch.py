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
        "front_cam": "1.2",
        "side_cam":  "1.3",
        "top_cam":   "1.4"
    }

    # Map the available video devices.
    devices = map_camera_devices(camera_mapping)
    for label, dev in devices.items():
        print(f"Mapping: {label} -> {dev}")

    # Get the device for each camera, defaulting if necessary.
    if "front_cam" in devices:
        front_cam_device = devices["front_cam"]
    else:
        front_cam_device = "/dev/video0"
        print("front_cam device not found in mapping, defaulting to /dev/video0")

    if "side_cam" in devices:
        side_cam_device = devices["side_cam"]
    else:
        side_cam_device = "/dev/video2"
        print("side_cam device not found in mapping, defaulting to /dev/video1")

    if "top_cam" in devices:
        top_cam_device = devices["top_cam"]
    else:
        top_cam_device = "/dev/video4"
        print("top_cam device not found in mapping, defaulting to /dev/video2")

    # Create a node for the front camera.
    node_front = Node(
        package="ugv_drivers",
        executable="UGVDriver.py",  # Adjust the executable name if necessary.
        name="change_exposure_front",
        namespace="front_cam",
        parameters=[
            {"video_device_index": front_cam_device},
            {"pipeline_port": 5000},
            {"ip_address": "192.168.0.100"},
            {"h": 480},
            {"w": 640},
            {"hz": 15},
        ],
        output="screen"
    )

    # Create a node for the side camera.
    node_side = Node(
        package="ugv_drivers",
        executable="UGVDriver.py",  # Adjust as needed.
        name="change_exposure_side",
        namespace="side_cam",
        parameters=[
            {"video_device_index": side_cam_device},
            {"pipeline_port": 5002},
            {"ip_address": "192.168.0.100"},
            {"w": 1920},
            {"h": 1080},
            {"hz": 15},
        ],
        output="screen"
    )

    # Create a node for the top camera.
    node_top = Node(
        package="ugv_drivers",
        executable="UGVDriver.py",  # Adjust as needed.
        name="change_exposure_top",
        namespace="top_cam",
        parameters=[
            {"video_device_index": top_cam_device},
            {"pipeline_port": 5004},
            {"ip_address": "192.168.0.100"},
            {"w": 1920},
            {"h": 1080},
            {"hz": 15},
        ],
        output="screen"
    )

    return [node_top]
    # Return all three nodes.
    # return [node_front, node_side, node_top]

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])

if __name__ == '__main__':
    generate_launch_description()
