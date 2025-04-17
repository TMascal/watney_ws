#!/home/mark/ros2-humble-env/bin/python3

import os
import cv2
import numpy as np

# Function to apply the color correction transform
def apply_color_correction(image, transform_matrix):
    # Convert image to RGB (if it's in BGR by default)
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    # Prepare the image pixels
    H, W, _ = image_rgb.shape
    pixels = image_rgb.reshape(-1, 3).astype(np.float32)

    # Augment pixels with a column of ones
    ones = np.ones((pixels.shape[0], 1), dtype=np.float32)
    pixels_aug = np.hstack([pixels, ones])   # Shape: (H*W, 4)

    # Apply the transformation
    corrected_pixels = np.dot(pixels_aug, transform_matrix)

    # Clip values to valid range and convert back to uint8
    corrected_pixels = np.clip(corrected_pixels, 0, 255).astype(np.uint8)

    # Reshape back to the image shape
    corrected_image = corrected_pixels.reshape(H, W, 3)

    # Convert the corrected image back to BGR
    corrected_image_bgr = cv2.cvtColor(corrected_image, cv2.COLOR_RGB2BGR)

    return corrected_image_bgr

# Function to crawl through directories and find the images
def process_images(input_dir, output_dir, transform_matrix):
    # Walk through all subdirectories in the input directory
    for root, dirs, files in os.walk(input_dir):
        for file in files:
            if file.endswith(".jpg") and file.startswith("basic_image"):
                image_path = os.path.join(root, file)
                print(f"Processing: {image_path}")

                # Load the image
                image = cv2.imread(image_path)

                if image is not None:
                    # Apply the color correction
                    corrected_image = apply_color_correction(image, transform_matrix)

                    # Define output path
                    output_path = os.path.join(output_dir, file)

                    # Save the corrected image
                    cv2.imwrite(output_path, corrected_image)
                    print(f"Saved corrected image to: {output_path}")
                else:
                    print(f"Error: The image {image_path} could not be loaded.")
        # Optionally, break after the first directory to avoid searching all subdirectories.
        # Remove the `break` if you want to process the whole directory tree.
        # break

# Load the color transform matrix from the saved .npy file
transform_matrix = np.load('/home/mark/watney_ws/pictures/Test1/color_transform_matrix.npy')

# Define input and output directories
input_dir = '/home/mark/Downloads/Top_Photos'
output_dir = '/home/mark/Pictures/color_corrected_images_static'

# Create output directory if it doesn't exist
os.makedirs(output_dir, exist_ok=True)

# Process all matching images
process_images(input_dir, output_dir, transform_matrix)

print("All images have been processed.")
