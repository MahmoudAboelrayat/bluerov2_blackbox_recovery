import cv2
import numpy as np
import glob
import os

# ==============================
# USER SETTINGS
# ==============================
image_dir = "/home/mahmoud/bluerov_ws/src/bluerov2_util/fames/calb"  # folder with your 12 images
image_pattern = "*.png"   # or "*.jpg" depending on format
chessboard_size = (9, 6)  # inner corners (columns, rows)
square_size = 0.055       # real size of each square (meters)
criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

# ==============================
# LOAD IMAGES
# ==============================
images = sorted(glob.glob(os.path.join(image_dir, image_pattern)))[:]
print(f"Found {len(images)} images")

if len(images) == 0:
    raise RuntimeError("No images found — check the folder path and file extension.")

# ==============================
# PREPARE OBJECT POINTS
# ==============================
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D points
imgpoints = []  # 2D points

# ==============================
# DETECT CHESSBOARD CORNERS
# ==============================
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret_corners, corners = cv2.findChessboardCorners(gray, chessboard_size, None)
    if ret_corners:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)

        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret_corners)
        cv2.imshow('Chessboard Detection', img)
        cv2.waitKey(300)
    else:
        print(f"❌ Chessboard not detected in {os.path.basename(fname)}")

cv2.destroyAllWindows()

# ==============================
# CALIBRATE CAMERA
# ==============================
if len(objpoints) < 5:
    raise RuntimeError("Not enough valid images with detected corners.")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

np.savez("camera_calibration.npz",
         camera_matrix=camera_matrix,
         dist_coeffs=dist_coeffs)

print("\n✅ Calibration complete")
print("Camera matrix:\n", camera_matrix)
print("Distortion coefficients:\n", dist_coeffs.ravel())
