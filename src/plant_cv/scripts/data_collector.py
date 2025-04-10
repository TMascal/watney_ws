#!/home/mark/ros2-humble-env/bin/python3

import cv2
import os

def main():
    # Define the folder path on your Desktop where photos will be saved.
    desktop_path = os.path.join(os.path.expanduser("~"), "Desktop")
    save_folder = os.path.join(desktop_path, "CapturedPhotos")
    if not os.path.exists(save_folder):
        os.makedirs(save_folder)
        print(f"Created folder: {save_folder}")

    # Open the default camera (0 is usually the built-in webcam).
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Cannot open camera")

        return

    img_counter = 0

    while True:
        # Capture frame-by-frame.
        ret, frame = cap.read()
        if not ret:
            print("Failed to grab frame")
            break

        # Display the live feed.
        cv2.imshow("Live Feed", frame)

        # Wait for key press for 1ms.
        key = cv2.waitKey(1)

        if key % 256 == 27:
            # ESC key pressed: exit the loop.
            print("Escape hit, closing...")
            break
        elif key % 256 == 32:
            # Spacebar pressed: save the current frame.
            img_name = f"photo_{img_counter}.png"
            save_path = os.path.join(save_folder, img_name)
            cv2.imwrite(save_path, frame)
            print(f"{img_name} written to {save_folder}!")
            img_counter += 1

    # When everything is done, release the capture and close windows.
    cap.release()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
