#!/home/mark/ros2-humble-env/bin/python3

# Tim Mascal
# 2/25/25

import cv2
import numpy as np

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()
K = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])  # Update K-matrix later
dist_coeffs = np.zeros((5, 1))
image_path = "/home/mark/watney_ws/pictures/imageSample.jpg"
image = cv2.imread(image_path)

if image is None:
    print(f"Error loading image from {image_path}")
else:
    detectorParams = cv2.aruco.DetectorParameters()
    detector = cv2.aruco.ArucoDetector(dictionary, detectorParams)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, rejected_candidates = detector.detectMarkers(gray)

    if ids is not None:
        for i in range(len(ids)):
            marker_corners = corners[i]
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            # rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.05, K, dist_coeffs)
            nada, rvec, tvec = cv2.solvePnP(marker_corners, 0.05, K, dist_coeffs, False, cv2.SOLVEPNP_IPPE_SQUARE)
            cv2.drawFrameAxes(image, K, dist_coeffs, rvec[0], tvec[0], 0.03)
            distance = np.linalg.norm(tvec[0][0])
            cX, cY = int(marker_corners[0][0][0]), int(marker_corners[0][0][1])
            cv2.putText(image, f"ID: {ids[i][0]} Dist: {distance:.2f}m", (cX, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    cv2.imshow('ArUco Tracker', image)

    cv2.waitKey(0)
cv2.destroyAllWindows()