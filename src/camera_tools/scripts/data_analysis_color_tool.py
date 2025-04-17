import cv2
import numpy as np
import os
import csv

# Function to calculate the average color within a given bounding box
def get_average_color(image, bbox):
    x, y, w, h = bbox
    roi = image[y:y+h, x:x+w]  # Region of interest (ROI) from the bounding box
    mean_color = np.mean(roi, axis=(0, 1))  # Calculate the mean color (BGR)
    return mean_color

# Callback function for drawing the bounding box
drawing = False  # Flag to indicate whether the mouse is being pressed
ix, iy = -1, -1  # Initial mouse coordinates
bbox = None  # Bounding box

def draw_bbox(event, x, y, flags, param):
    global ix, iy, drawing, bbox

    if event == cv2.EVENT_LBUTTONDOWN:
        drawing = True
        ix, iy = x, y

    elif event == cv2.EVENT_MOUSEMOVE:
        if drawing:
            img_copy = img.copy()
            cv2.rectangle(img_copy, (ix, iy), (x, y), (0, 255, 0), 2)
            cv2.imshow("Image", img_copy)

    elif event == cv2.EVENT_LBUTTONUP:
        drawing = False
        bbox = (ix, iy, x - ix, y - iy)  # x, y, width, height
        cv2.rectangle(img, (ix, iy), (x, y), (0, 255, 0), 2)
        cv2.imshow("Image", img)
        print(f"Bounding box: {bbox}")

# Function to find the original image corresponding to the color-corrected image
def find_original_image(color_corrected_image_name):
    # Recursively search through the top and side directories
    possible_paths = [
        os.path.join('/home/mark/Pictures/top'),
        os.path.join('/home/mark/Pictures/side')
    ]
    for path in possible_paths:
        for root, dirs, files in os.walk(path):
            if color_corrected_image_name in files:
                return cv2.imread(os.path.join(root, color_corrected_image_name))
    return None

# Main function to process the images
def process_images(input_dir, corrected_images_dir, output_csv):
    color_data = []  # List to store the average color values
    images = [f for f in os.listdir(corrected_images_dir) if f.endswith(".jpg")]

    for image_name in images:
        corrected_image_path = os.path.join(corrected_images_dir, image_name)
        print(f"Processing: {corrected_image_path}")

        global img, bbox
        img = cv2.imread(corrected_image_path)
        if img is None:
            print(f"Error: Could not load corrected image {image_name}. Skipping.")
            continue

        # Find corresponding original image
        original_image = find_original_image(image_name)
        if original_image is None:
            print(f"Error: Could not find corresponding original image for {image_name}. Skipping.")
            continue

        # Set up the mouse callback to draw bounding boxes
        cv2.namedWindow("Image")
        cv2.setMouseCallback("Image", draw_bbox)

        while True:
            cv2.imshow("Image", img)

            key = cv2.waitKey(1) & 0xFF

            # If the user presses 'p', skip this image
            if key == ord('p'):
                print(f"Skipping image: {image_name}")
                break

            # If the user presses 'esc', save the color data and move to the next image
            if key == 27:  # ESC key
                if bbox is not None:
                    # Get the average color of the bounding box for both images
                    avg_color_corrected = get_average_color(img, bbox)
                    avg_color_original = get_average_color(original_image, bbox)
                    color_data.append([image_name, avg_color_original[0], avg_color_original[1], avg_color_original[2],
                                       avg_color_corrected[0], avg_color_corrected[1], avg_color_corrected[2]])
                    print(f"Added color data for {image_name}: Original {avg_color_original} | Corrected {avg_color_corrected}")
                else:
                    print(f"No bounding box selected for {image_name}. Skipping.")
                break

        # Close the window
        cv2.destroyAllWindows()

    # Save the color data to the CSV file
    with open(output_csv, mode='w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(['Image Name', 'Original B', 'Original G', 'Original R', 'Corrected B', 'Corrected G', 'Corrected R'])
        writer.writerows(color_data)
    print(f"Color data saved to {output_csv}")

# Input and output directories
input_dir = '/home/mark/Pictures'  # Update with the correct path
corrected_images_dir = '/home/mark/Pictures/color_corrected_images'  # Color-corrected images directory
output_csv = 'color_data.csv'  # CSV file to save the color data

# Process the images and save the color data
process_images(input_dir, corrected_images_dir, output_csv)
