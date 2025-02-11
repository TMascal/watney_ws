#!/home/mark/ros2-humble-env/bin/python3

import cv2
import os

# Ensure the 'pictures' folder exists
pictures_folder = "pictures"
if not os.path.exists(pictures_folder):
    os.makedirs(pictures_folder)

# Initialize the webcam
camera = cv2.VideoCapture(2)

if not camera.isOpened():
    print("Error: Unable to access the camera.")
    exit()

print("Press 's' to take a picture and save it, or 'q' to quit.")

while True:
    # Capture frame-by-frame
    ret, frame = camera.read()
    if not ret:
        print("Error: Unable to capture video.")
        break

    # Show the video feed
    cv2.imshow("Camera Feed", frame)

    # Wait for key press
    key = cv2.waitKey(1) & 0xFF

    if key == ord('s'):
        # Save the picture
        picture_name = os.path.join(pictures_folder, "captured_image.jpg")
        cv2.imwrite(picture_name, frame)
        print(f"Picture saved at {picture_name}")
        break

    elif key == ord('q'):
        # Quit without saving
        print("Quitting...")
        break

# Release the camera and close all OpenCV windows
camera.release()
cv2.destroyAllWindows()