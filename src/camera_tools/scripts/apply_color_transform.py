#!/home/mark/ros2-humble-env/bin/python3

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

# Load the color transform matrix from the saved .npy file
transform_matrix = np.load('/home/mark/watney_ws/pictures/Test1/color_transform_matrix.npy')

# Read the input image
image_path = '/home/mark/Pictures/top/Good_2025-04-16_14-09-42/hdr_image_2025-04-16_14-09-42.jpg'  # Update with your image path
image = cv2.imread(image_path)

if image is not None:
    # Apply the color correction
    corrected_image = apply_color_correction(image, transform_matrix)

    # Save the color-corrected image
    corrected_image_path = '/home/mark/watney_ws/pictures/Test1/corrected_hdr_image.jpg'  # Update with your desired output path
    cv2.imwrite(corrected_image_path, corrected_image)

    print(f"Color-corrected image saved to {corrected_image_path}")
else:
    print("Error: The image could not be loaded. Please check the image path.")
