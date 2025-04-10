#!/home/pi/ros2-humble-env/bin/python3

import cv2
import numpy as np
import numpy as np
from scipy.spatial.distance import cdist

dictionary = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_4X4_1000)
parameters = cv2.aruco.DetectorParameters()
K = np.array([[1000, 0, 320], [0, 1000, 240], [0, 0, 1]])  # Update K-matrix later
dist_coeffs = np.zeros((5, 1))
# image_path = '/home/mark/watney_ws/pictures/captured_image.jpg'
image_path = '/home/mark/Downloads/labtest.jpg'
image = cv2.imread(image_path)
unedited = image.copy()


if image is not None:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=parameters)

    if ids is not None:
        for i in range(len(ids)):
            marker_corners = corners[i]
            cv2.aruco.drawDetectedMarkers(image, corners, ids)
            rvec, tvec, _ = cv2.aruco.estimatePoseSingleMarkers(marker_corners, 0.05, K, dist_coeffs)
            cv2.drawFrameAxes(image, K, dist_coeffs, rvec[0], tvec[0], 0.03)
            distance = np.linalg.norm(tvec[0][0])
            cX, cY = int(marker_corners[0][0][0]), int(marker_corners[0][0][1])
            cv2.putText(image, f"ID: {ids[i][0]} Dist: {distance:.2f}m", (cX, cY - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 255, 0), 2)

x1, y1 = (corners[1])[-1, 1, :]
x3, y3 = (corners[0])[-1, 0, :]
x2, y2 = x3, y1
x4, y4 = x1, y3

pts_src = np.array([[x1, y1], [x2, y2], [x3, y3], [x4, y4]], dtype="float32")
width = int(max(np.linalg.norm(pts_src[0] - pts_src[1]), np.linalg.norm(pts_src[2] - pts_src[3])))
height = int(max(np.linalg.norm(pts_src[0] - pts_src[3]), np.linalg.norm(pts_src[1] - pts_src[2])))
pts_dst = np.array([[0, 0], [width - 1, 0], [width - 1, height - 1], [0, height - 1]], dtype="float32")
matrix = cv2.getPerspectiveTransform(pts_src, pts_dst)
warped = cv2.warpPerspective(unedited, matrix, (width, height))

# Get image dimensions (height, width, channels)
height, width = warped.shape[:2]

# Define your crop percentages
# For example, crop to the central 80% of the image
x_start_perc, x_end_perc = 0.20, 0.80  # 10% to 90% of the width
y_start_perc, y_end_perc = 0.00, 0.88  # 10% to 90% of the height

# Calculate pixel indices
x_start = int(width * x_start_perc)
x_end = int(width * x_end_perc)
y_start = int(height * y_start_perc)
y_end = int(height * y_end_perc)

# Crop the image using NumPy slicing
cropped_image = warped[y_start:y_end, x_start:x_end]

_, bw_image = cv2.threshold(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2GRAY), 128, 255, cv2.THRESH_BINARY)

# Setup SimpleBlobDetector parameters.
params = cv2.SimpleBlobDetector_Params()
params.filterByColor = False
params.blobColor = 0  # Looking for dark (black) blobs
params.filterByArea = True
params.minArea = 25  # adjust based on your blob size

# Create a detector with the parameters
detector = cv2.SimpleBlobDetector_create(params)
# Load image and detect blobs
keypoints = detector.detect(cropped_image)
print("Detected blobs:", len(keypoints))

# Setup: Compute Expected Grid Centers
# Assume keypoints have already been detected and are stored in 'keypoints'.
# Convert keypoints to an array of (x, y) coordinates.
points = np.array([kp.pt for kp in keypoints])

# Define grid dimensions: 6 columns and 4 rows (total 24 cells).
num_cols = 6
num_rows = 4

# Determine the bounding box of the detected keypoints.
x_min, y_min = points.min(axis=0)
x_max, y_max = points.max(axis=0)

# Compute expected center positions for each grid cell.
x_centers = np.linspace(x_min, x_max, num_cols)
y_centers = np.linspace(y_min, y_max, num_rows)
# Create grid centers in row-major order (cell 0: row 0, col 0; cell 1: row 0, col 1; etc.)
grid_centers = np.array([(x, y) for y in y_centers for x in x_centers])

# Step 1: Assign Detected Keypoints to Nearest Grid Cells
# Compute distances from each detected keypoint to all grid centers.
distances = cdist(points, grid_centers)
# For each keypoint, find the nearest grid center index.
assigned_indices = np.argmin(distances, axis=1)

# Create a list to hold the keypoint (or inferred coordinate) for each grid cell.
# Initialize with None so we can detect missing ones.
grid_keypoints = [None] * (num_rows * num_cols)

# For each detected keypoint, assign it to the grid cell.
# If more than one keypoint gets assigned to the same cell, keep the one closest to the expected center.
for i, kp in enumerate(keypoints):
    grid_idx = assigned_indices[i]
    current_pt = np.array(kp.pt)
    if grid_keypoints[grid_idx] is None:
        grid_keypoints[grid_idx] = kp
    else:
        # Compare distances to the expected grid center.
        prev_pt = np.array(grid_keypoints[grid_idx].pt)
        if np.linalg.norm(current_pt - grid_centers[grid_idx]) < np.linalg.norm(prev_pt - grid_centers[grid_idx]):
            grid_keypoints[grid_idx] = kp

# Step 2: Infer Missing Keypoints Using Consistent Spacing
for idx in range(len(grid_keypoints)):
    if grid_keypoints[idx] is None:
        row = idx // num_cols
        col = idx % num_cols

        # Gather neighbor coordinates (using detected keypoints) from left, right, top, and bottom.
        neighbors = []
        x_dim = []
        y_dim = []
        # Left neighbor
        if col > 0 and grid_keypoints[idx - 1] is not None:
            left_pts = np.array(grid_keypoints[idx - 1].pt)
            neighbors.append(left_pts)
            y_dim.append(left_pts[1])

        # Right neighbor
        if col < num_cols - 1 and grid_keypoints[idx + 1] is not None:
            right_pts = np.array(grid_keypoints[idx + 1].pt)
            neighbors.append(right_pts)
            y_dim.append(right_pts[1])

        # Top neighbor
        if row > 0 and grid_keypoints[idx - num_cols] is not None:
            top_pts = np.array(grid_keypoints[idx - num_cols].pt)
            neighbors.append(top_pts)
            x_dim.append(top_pts[0])

        # Bottom neighbor
        if row < num_rows - 1 and grid_keypoints[idx + num_cols] is not None:
            btm_pts = np.array(grid_keypoints[idx + num_cols].pt)
            neighbors.append(btm_pts)
            x_dim.append(btm_pts[0])

        if neighbors:
            if len(x_dim) > 1:
                x_pt = np.mean(x_dim)
            elif len(x_dim) == 1:
                x_pt = x_dim[0]

            if len(y_dim) > 1:
                y_pt = np.mean(y_dim)
            elif len(y_dim) == 1:
                y_pt = y_dim[0]

            inferred_pt = np.array([x_pt, y_pt])

        else:
            # If no neighbors are available, fall back to the expected grid center.
            inferred_pt = grid_centers[idx]

        # For consistency, store the inferred point as a cv2.Keyponint
        if keypoints:
            average_size = np.mean([kp.size for kp in keypoints])
        else:
            average_size = 1.0  # Fallback value if keypoints is empty
        inferred_keypoint = cv2.KeyPoint(x=float(inferred_pt[0]), y=float(inferred_pt[1]), size=average_size)
        grid_keypoints[idx] = inferred_keypoint

# DataBank
colorchecker_rgb = np.array([
    [115,  82,  68],  # Dark Skin
    [194, 150, 130],  # Light Skin
    [ 98, 122, 157],  # Blue Sky
    [ 87, 108,  67],  # Foliage
    [133, 128, 177],  # Blue Flower
    [103, 189, 170],  # Bluish Green
    [214, 126,  44],  # Orange
    [ 80,  91, 166],  # Purplish Blue
    [193,  90,  99],  # Moderate Red
    [ 94,  60, 108],  # Purple
    [157, 188,  64],  # Yellow Green
    [224, 163,  46],  # Orange Yellow
    [ 56,  61, 150],  # Blue
    [ 70, 148,  73],  # Green
    [175,  54,  60],  # Red
    [231, 199,  31],  # Yellow
    [187,  86, 149],  # Magenta
    [  8, 133, 161],  # Cyan
    [243, 243, 242],  # White
    [200, 200, 200],  # Neutral 8
    [160, 160, 160],  # Neutral 6.5
    [122, 122, 121],  # Neutral 5
    [ 85,  85,  85],  # Neutral 3.5
    [ 52,  52,  52]   # Black
], dtype=np.uint8)

def get_average_rgb(image, center, radius):
    """
    Given an image, a center (x, y), and a radius,
    returns the average RGB (or BGR) color within that circular region.
    """
    # Create a mask of zeros (same height and width as the image)
    mask = np.zeros(image.shape[:2], dtype=np.uint8)
    # Draw a filled circle on the mask (white=255) at the given center and radius
    cv2.circle(mask, center, radius, 255, thickness=-1)

    # Use cv2.mean to compute the average color in the masked region.
    # Note: If your image is in BGR format (default for cv2.imread),
    # cv2.mean returns (B, G, R, A).
    mean_color = cv2.mean(image, mask=mask)

    # Return only the first three channels (B, G, R)
    return mean_color[:3]

# Sample the color at the centroid
sampled_color = np.zeros((24,3))
for i in range(len(grid_keypoints)):
    kp = np.array((grid_keypoints[i].pt)).astype(int)
    size = grid_keypoints[i].size
    avg_color = get_average_rgb(cv2.cvtColor(cropped_image, cv2.COLOR_BGR2RGB), kp, round(size/3))
    sampled_color[i, :] = avg_color

# Add a column of ones to measured to account for the offset.
ones = np.ones((sampled_color.shape[0], 1), dtype=np.float32)
A = np.hstack([sampled_color, ones])   # Now A is (N, 4)

T, residuals, rank, s = np.linalg.lstsq(A, colorchecker_rgb, rcond=None)

image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

# Prepare the image pixels.
H, W, _ = image_rgb.shape
pixels = image_rgb.reshape(-1, 3).astype(np.float32)

# Augment pixels with a column of ones.
ones = np.ones((pixels.shape[0], 1), dtype=np.float32)
pixels_aug = np.hstack([pixels, ones])   # Shape: (H*W, 4)

# Apply the transformation.
corrected_pixels = np.dot(pixels_aug, T)

# Clip values to valid range and convert back to uint8.
corrected_pixels = np.clip(corrected_pixels, 0, 255).astype(np.uint8)

# Reshape back to the image shape.
corrected_image = corrected_pixels.reshape(H, W, 3)

corrected_image = cv2.cvtColor(corrected_image, cv2.COLOR_RGB2BGR)


img_with_keypoints = cv2.drawKeypoints(cropped_image, grid_keypoints, None, (0, 0, 255), cv2.DRAW_MATCHES_FLAGS_DRAW_RICH_KEYPOINTS)

cv2.imshow('ArUco Tracker', image)
cv2.imshow('Cropped Image', bw_image)
cv2.imshow('Keypoints', img_with_keypoints)
cv2.imshow('Corrected Image', corrected_image)
cv2.waitKey(0)
cv2.destroyAllWindows()




