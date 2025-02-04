import cv2
import numpy as np


dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_50)
parameters = cv2.aruco.DetectorParameters()
K = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])  # Update K-matrix later
dist_coeffs = np.zeros((5, 1))
cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break
    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)
    
    if ids is not None:
        for i in range(len(ids)):
            marker_corners = corners[i]
            cv2.aruco.drawDetectedMarkers(frame, corners, ids)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.05, K, dist_coeffs)
            cv2.drawFrameAxes(frame, K, dist_coeffs, rvec[0], tvec[0], 0.03)
            cX, cY = int(marker_corners[0][0][0]), int(marker_corners[0][0][1])
            cv2.putText(frame, f"Frame {ids[i][0]}", (cX, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    cv2.imshow('ArUco Tracker', frame)
    
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
