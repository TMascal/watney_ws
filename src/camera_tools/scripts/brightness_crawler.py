import os
import csv
import cv2
import numpy as np

def calculate_brightness(image_path):
    try:
        # Load the image using OpenCV
        img = cv2.imread(image_path)
        if img is None:
            print(f"Warning: Unable to load image {image_path}")
            return None

        # Convert image to grayscale
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Calculate the brightness (mean of the grayscale image)
        brightness = np.mean(gray)
        print(f"Brightness for {image_path}: {brightness}")
        return brightness
    except Exception as e:
        print(f"Error calculating brightness for {image_path}: {e}")
        return None

def scan_images(directory):
    # Store results for brightness values
    brightness_results = []

    # Walk through the directory and its subdirectories
    print(f"Scanning directory: {directory}")
    for root, dirs, files in os.walk(directory):
        print(f"Scanning directory: {root}... Found {len(files)} files")

        for file in files:
            # Reject files that end with "high.jpg", "mid.jpg", or "low.jpg"
            if file.endswith(('high.jpg', 'mid.jpg', 'low.jpg')):
                print(f"Rejecting file: {file}")
                continue

            if file.endswith('.jpg') and (file.startswith('basic_image') or file.startswith('hdr_image')):
                image_path = os.path.join(root, file)
                print(f"Processing image: {image_path}")
                brightness_value = calculate_brightness(image_path)

                if brightness_value is None:
                    print(f"Skipping image {image_path} due to brightness calculation error")
                    continue

                # Record brightness
                brightness_results.append((image_path, brightness_value))

    # Save the results to a CSV file
    print(f"Saving results to 'brightness_values.csv'...")
    with open('brightness_values.csv', mode='w', newline='') as csvfile:
        fieldnames = ['image_path', 'brightness_value']
        writer = csv.DictWriter(csvfile, fieldnames=fieldnames)

        writer.writeheader()
        for image_path, brightness_value in brightness_results:
            writer.writerow({'image_path': image_path, 'brightness_value': brightness_value})

    print(f"Processed {len(brightness_results)} images. Results saved to 'brightness_values.csv'.")

def main():
    # Specify the directory to scan
    scan_images('/home/mark/Pictures/top')

if __name__ == '__main__':
    main()
