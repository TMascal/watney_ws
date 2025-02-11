#!/home/mark/ros2-humble-env/bin/python3

import cv2
import os
import numpy as np

# Configurable directory containing the photos
photos_directory = "/home/mark/Pictures/hdr"  # Change this to your desired directory

# Exposure times in seconds
exposure_times = np.array([0.01, 0.1, 0.2], dtype=np.float32)  # Converted from 100ms, 1000ms, and 2000ms


def merge_hdr_images(directory):
    # Collect image paths
    image_files = [os.path.join(directory, f) for f in sorted(os.listdir(directory)) if
                   f.endswith((".jpg", ".png", ".jpeg"))]

    # Check if at least three images are present
    if len(image_files) < 3:
        print(
            "Error: Less than three images found in the directory. Please ensure the directory contains at least three photos.")
        return

    # Read the images into a list
    images = [cv2.imread(img) for img in image_files[:3]]  # Take the first 3 images

    # Check if any image failed to load
    if any(image is None for image in images):
        print("Error: Unable to read one or more images. Please check the image files.")
        return

    # Ensure the number of exposure times matches the number of images
    if len(images) != len(exposure_times):
        print("Error: The number of exposure times does not match the number of input images.")
        return

    # Merge the images into an HDR photo
    try:
        # Convert to HDR using OpenCV
        merge_debevec = cv2.createMergeDebevec()
        hdr_image = merge_debevec.process(images, times=exposure_times)

        # Tone mapping for display (converting HDR to 8-bit)
        # Use cv2.createTonemap() (generic tone-mapper) instead of cv2.createTonemapDurand
        # tonemap = cv2.createTonemap(gamma=2.2)  # Generic gamma correction tone-mapper
        tonemap = cv2.createTonemapReinhard(gamma=1.5, intensity=0, light_adapt=0, color_adapt=1.0)
        # tonemap = cv2.createTonemapDrago(gamma=1.2, saturation=1.0, bias=0.85)
        ldr_image = tonemap.process(hdr_image)
        ldr_image = cv2.normalize(ldr_image, None, 0, 255, cv2.NORM_MINMAX, dtype=cv2.CV_8UC3)

        # Save the resulting image
        output_path = os.path.join(directory, "result_hdr_image.jpg")
        cv2.imwrite(output_path, ldr_image)
        print(f"HDR image successfully saved at: {output_path}")
    except Exception as e:
        print(f"Error while generating HDR image: {e}")


if __name__ == "__main__":
    merge_hdr_images(photos_directory)