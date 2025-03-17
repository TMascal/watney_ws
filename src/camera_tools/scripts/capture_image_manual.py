#!/home/mark/ros2-humble-env/bin/python3

# Tim Mascal
# 2/25/25

import cv2

# Read the image from the specified path
save_path = '/home/mark/watney_ws/pictures/captured_image.jpg'

camera = cv2.VideoCapture(3, cv2.CAP_V4L2)
while True:
    ret, frame = camera.read()
    if not ret or frame is None:
        print("Error: Unable to capture image from camera")
        break
    cv2.imshow("Captured Image", frame)
    key = cv2.waitKey(1) & 0xFF
    if key == ord('s'):
        cv2.imwrite(save_path, frame)
        print(f"Image saved to {save_path}")
        break

cv2.destroyAllWindows()
camera.release()
