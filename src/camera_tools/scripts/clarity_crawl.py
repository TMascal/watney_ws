#!/home/mark/ros2-humble-env/bin/python3

import os
import csv
import cv2
from dom import DOM

def calculate_clarity(image_path):
    try:
        # Load the image using OpenCV
        img = cv2.imread(image_path)
        if img is None:
            print(f"Warning: Unable to load image {image_path}")
            return None

        # Initialize the DOM object
        iqa = DOM()

        # Calculate clarity using the get_sharpness method
        clarity_value = iqa.get_sharpness(img)
        print(f"Clarity value for {image_path}: {clarity_value}")
        return clarity_value
    except Exception as e:
        print(f"Error calculating clarity for {image_path}: {e}")
        return None

def scan_images(directory):
    # Store results for clarity values
    clarity_results = []

    # Walk through the directory and its subdirectories
    print(f"Scanning directory: {directory}")
    for root, dirs, files in os.walk(directory):
        print(f"Scanning directory: {root}... Found {len(files)} files")

        for file in files:
            if file.endswith('.jpg'):
                image_path = os.path.join(root, file)
                print(f"Processing image: {image_path}")
                clarity_value = calculate_clarity(image_path)

                if clarity_value is None:
                    print(f"Skipping image {image_path} due to clarity calculation error")
                    continue

                # Discard values below 0.7
                if clarity_value >= 0.7:
                    clarity_results.append((image_path, clarity_value))
                else:
                    print(f"Discarding {image_path} with clarity value: {clarity_value}")

    # Save the results to a CSV file
    print(f"Saving results to 'clarity_values.csv'...")
    with open('clarity_values.csv', mode='w', newline='') as csvfile:
        fieldnames = ['image_path', 'clarity_value']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for image_path, clarity_value in clarity_results:
            writer.writerow({'image_path': image_path, 'clarity_value': clarity_value})

    print(f"Processed {len(clarity_results)} images. Results saved to 'clarity_values.csv'.")

def main():
    # Specify the directory to scan
    scan_images('/home/mark/Pictures/top')

if __name__ == '__main__':
    main()
