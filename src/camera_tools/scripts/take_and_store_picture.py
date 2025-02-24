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

desired_width = 2592  # Hardcoded width
desired_height = 1944  # Hardcoded height
camera.set(cv2.CAP_PROP_FRAME_WIDTH, desired_width)
camera.set(cv2.CAP_PROP_FRAME_HEIGHT, desired_height)

print("Press 's' to take a picture and save it, or 'q' to quit.")

ret, frame = camera.read()
cv2.imwrite(f'//home//mark//watney_ws//pictures//image_{6}.jpg', frame)

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