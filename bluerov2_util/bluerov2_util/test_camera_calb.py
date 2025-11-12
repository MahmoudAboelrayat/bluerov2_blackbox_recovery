import cv2
import numpy as np

# Load saved calibration
data = np.load("camera_calibration.npz")
camera_matrix = data["camera_matrix"]
dist_coeffs = data["dist_coeffs"]

print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs.ravel())


import glob

image_dir = "/home/mahmoud/bluerov_ws/src/bluerov2_util/fames/calb"
images = sorted(glob.glob(f"{image_dir}/*.png"))

for fname in images[:12]:  # test on the first 3 images
    img = cv2.imread(fname)
    h, w = img.shape[:2]
    
    # Optional: compute optimal new camera matrix to reduce black borders
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )

    # Undistort
    undistorted = cv2.undistort(img, camera_matrix, dist_coeffs, None, new_camera_matrix)

    # Crop the image if you used ROI
    x, y, w, h = roi
    undistorted = undistorted[y:y+h, x:x+w]

    # Show results
    cv2.imshow("Original", img)
    cv2.imshow("Undistorted", undistorted)
    cv2.waitKey(0)

cv2.destroyAllWindows()
