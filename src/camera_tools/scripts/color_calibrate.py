#!/home/mark/ros2-humble-env/bin/python3

# Tim Mascal
# 2/25/25

import cv2
import numpy as np

scene_path = "//home//mark//watney_ws//pictures//image.jpg"
# Load reference (template) and scene images
ref_img = cv2.imread("//home//mark//Pictures//ColorCalTest//Reference Photo//Reference.jpg", cv2.IMREAD_GRAYSCALE)  # Object to detect
scene_img = cv2.imread(scene_path, cv2.IMREAD_GRAYSCALE)  # Image where object is located

# Initialize ORB detector
orb = cv2.ORB_create(nfeatures=1000)

# Detect and compute keypoints and descriptors
kp1, des1 = orb.detectAndCompute(ref_img, None)
kp2, des2 = orb.detectAndCompute(scene_img, None)

# Use Brute-Force matcher with Hamming distance (ORB uses binary descriptors)
bf = cv2.BFMatcher(cv2.NORM_HAMMING, crossCheck=True)
matches = bf.match(des1, des2)

# Sort matches based on distance (lower is better)
matches = sorted(matches, key=lambda x: x.distance)

# Draw top matches for visualization
img_matches = cv2.drawMatches(ref_img, kp1, scene_img, kp2, matches[:50], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

# Object localization using Homography
good_matches = matches[:30]  # Select top 30 matches
if len(good_matches) > 10:
    src_pts = np.float32([kp1[m.queryIdx].pt for m in good_matches]).reshape(-1, 1, 2)
    dst_pts = np.float32([kp2[m.trainIdx].pt for m in good_matches]).reshape(-1, 1, 2)

    # Compute Homography matrix
    M, mask = cv2.findHomography(src_pts, dst_pts, cv2.RANSAC, 5.0)

    # Get the object's bounding box
    h, w = ref_img.shape
    pts = np.float32([[0, 0], [w, 0], [w, h], [0, h]]).reshape(-1, 1, 2)
    dst = cv2.perspectiveTransform(pts, M)

    # Load scene in color to draw bounding box
    scene_img_color = cv2.imread(scene_path)

    # Draw the bounding box on the detected object
    cv2.polylines(scene_img_color, [np.int32(dst)], True, (0, 255, 0), 3, cv2.LINE_AA)

    # Also draw bounding box on the matched keypoints visualization
    img_matches_color = cv2.drawMatches(ref_img, kp1, scene_img_color, kp2, matches[:50], None, flags=cv2.DrawMatchesFlags_NOT_DRAW_SINGLE_POINTS)

    # Resize images to fit the screen dynamically
    screen_res = (1280, 720)  # Adjust this based on your screen size
    scale_percent = min(screen_res[0] / scene_img_color.shape[1], screen_res[1] / scene_img_color.shape[0])

    new_size = (int(scene_img_color.shape[1] * scale_percent), int(scene_img_color.shape[0] * scale_percent))
    resized_scene = cv2.resize(scene_img_color, new_size, interpolation=cv2.INTER_AREA)

    new_size_matches = (int(img_matches_color.shape[1] * scale_percent), int(img_matches_color.shape[0] * scale_percent))
    resized_matches = cv2.resize(img_matches_color, new_size_matches, interpolation=cv2.INTER_AREA)

    # Set window to resizable mode
    cv2.namedWindow("Matches & Bounding Box", cv2.WINDOW_NORMAL)
    cv2.namedWindow("Detected Object", cv2.WINDOW_NORMAL)

    # Display images
    cv2.imshow("Matches & Bounding Box", resized_matches)
    cv2.imshow("Detected Object", resized_scene)

    cv2.waitKey(0)
    cv2.destroyAllWindows()
else:
    print("Not enough good matches to find the object.")