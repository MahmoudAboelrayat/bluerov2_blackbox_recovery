import cv2
import numpy as np
import glob
import os

# ==============================
# USER SETTINGS
# ==============================
image_dir = "/media/mahmoud/DATA/calibration_11_11/frames"  # folder with images
image_pattern = "*.png"   # "*.jpg" if your images are JPG
chessboard_size = (9, 6)  # inner corners (cols, rows)
square_size = 0.055       # each square size in meters
criteria = (
    cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
    30, 
    0.001
)

# ==============================
# LOAD IMAGES
# ==============================
images = sorted(glob.glob(os.path.join(image_dir, image_pattern)))
print(f"Found {len(images)} images\n")

if len(images) == 0:
    raise RuntimeError("No calibration images found. Check folder path and extension.")

# ==============================
# PREPARE OBJECT POINTS (3D points in real world)
# ==============================
objp = np.zeros((chessboard_size[0]*chessboard_size[1], 3), np.float32)
objp[:, :2] = np.mgrid[0:chessboard_size[0], 0:chessboard_size[1]].T.reshape(-1, 2)
objp *= square_size

objpoints = []  # 3D points for each image
imgpoints = []  # 2D points for each image

# ==============================
# DETECT CORNERS
# ==============================
for fname in images:
    img = cv2.imread(fname)
    gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

    ret_corners, corners = cv2.findChessboardCorners(gray, chessboard_size, None)

    if ret_corners:
        corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
        objpoints.append(objp)
        imgpoints.append(corners2)

        # Draw and view results
        cv2.drawChessboardCorners(img, chessboard_size, corners2, ret_corners)
        cv2.imshow("Chessboard Detection", img)
        cv2.waitKey(150)
    else:
        print(f"❌ No corners detected in: {os.path.basename(fname)}")

cv2.destroyAllWindows()

if len(objpoints) < 5:
    raise RuntimeError("Not enough valid images with detected corners for calibration.")

# ==============================
# CAMERA CALIBRATION
# ==============================
print("\n🔧 Running camera calibration...")

ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
    objpoints, imgpoints, gray.shape[::-1], None, None
)

# ==============================
# COMPUTE REPROJECTION ERROR
# ==============================
total_error = 0
errors = []

for i in range(len(objpoints)):
    imgpoints_proj, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], camera_matrix, dist_coeffs)
    error = cv2.norm(imgpoints[i], imgpoints_proj, cv2.NORM_L2) / len(imgpoints_proj)
    errors.append(error)
    total_error += error

mean_error = total_error / len(objpoints)

print("\n==============================")
print("📌  PER-IMAGE REPROJECTION ERRORS")
print("==============================")
for i, e in enumerate(errors):
    print(f"Image {i+1}:   {e:.4f} pixels")

print("\n==============================")
print("📌  FINAL MEAN REPROJECTION ERROR")
print("==============================")
print(f"➡️  Mean Reprojection Error = {mean_error:.4f} pixels")
print("==============================\n")

# ==============================
# SAVE CALIBRATION
# ==============================
np.savez("camera_calibration_16_11.npz",
         camera_matrix=camera_matrix,
         dist_coeffs=dist_coeffs)

print("✅ Calibration saved to camera_calibration.npz\n")
print("Camera Matrix:\n", camera_matrix)
print("Distortion Coefficients:\n", dist_coeffs.ravel())
