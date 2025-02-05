import cv2
import numpy as np

image_path = "/home/suave/TMP/plant.jpg"
image = cv2.imread(image_path)
if image is None:
    print("Error: Could not read image.")
    exit()

image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
lower_green = np.array([25, 40, 40])  # Lower bound of green in HSV
upper_green = np.array([90, 255, 255])  # Upper bound of green in HSV

mask = cv2.inRange(hsv, lower_green, upper_green)
contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

if contours:
    largest_contour = max(contours, key=cv2.contourArea)
    x, y, w, h = cv2.boundingRect(largest_contour)
    cv2.rectangle(image, (x, y), (x + w, y + h), (255, 0, 0), 3)
    plant_size = (w, h)
else:
    plant_size = (0, 0)

cv2.imshow(f"Bounding Box: {plant_size[0]} x {plant_size[1]} pixels", image)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(f"Detected Plant Size: {plant_size[0]} x {plant_size[1]} pixels")
