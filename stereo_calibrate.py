#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import os
import yaml


def main():
    # ============= 1. Set chessboard parameters =============
    # Note: You should modify according to your actual chessboard inner corners and square size
    pattern_size = (9, 6)  # Number of inner corners (columns, rows), keep 9x6 if you are using it
    square_size = 0.010  # Square size (meters), e.g., 1cm
    criteria_subpix = (
        cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER,
        100,
        1e-6
    )

    # ============= 2. Prepare 3D points of chessboard in world coordinates =============
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # ============= 3. Load captured calibration images =============
    # Your original path: ~/stereo_calib_images
    # Replace according to your actual path
    image_dir = os.path.expanduser("~/stereo_calib_images")
    left_images = sorted(glob.glob(os.path.join(image_dir, "left_*.png")))
    right_images = sorted(glob.glob(os.path.join(image_dir, "right_*.png")))

    if len(left_images) == 0 or len(right_images) == 0:
        print(f"Not enough image files found in directory {image_dir}.")
        return
    assert len(left_images) == len(right_images), "Mismatched number of left and right images!"

    # Prepare to store corner points
    objpoints = []
    imgpoints_left = []
    imgpoints_right = []

    # ============= 4. Loop through images to extract corners =============
    for lf, rf in zip(left_images, right_images):
        imgL_raw = cv2.imread(lf)
        imgR_raw = cv2.imread(rf)
        if imgL_raw is None or imgR_raw is None:
            print(f"Skipping empty file: {lf}, {rf}")
            continue

        grayL = cv2.cvtColor(imgL_raw, cv2.COLOR_BGR2GRAY)
        grayR = cv2.cvtColor(imgR_raw, cv2.COLOR_BGR2GRAY)

        retL, cornersL = cv2.findChessboardCorners(
            grayL, pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        )
        retR, cornersR = cv2.findChessboardCorners(
            grayR, pattern_size,
            flags=cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE + cv2.CALIB_CB_FAST_CHECK
        )

        if retL and retR:
            # Further refine to subpixel accuracy
            cornersL = cv2.cornerSubPix(grayL, cornersL, (11, 11), (-1, -1), criteria_subpix)
            cornersR = cv2.cornerSubPix(grayR, cornersR, (11, 11), (-1, -1), criteria_subpix)

            objpoints.append(objp)
            imgpoints_left.append(cornersL)
            imgpoints_right.append(cornersR)
        else:
            print(f"Chessboard corners not detected: {lf} or {rf}")

    if not objpoints:
        print("Failed to detect any chessboard corners, calibration cannot proceed!")
        return

    # Assume all images have same size, get size from any image
    image_size = grayL.shape[::-1]
    print(f"Image size: {image_size}")

    # ============= 5. First monocular calibration (left and right cameras) =============
    ret_left, M1, d1, rvecs1, tvecs1 = cv2.calibrateCamera(
        objpoints, imgpoints_left, image_size, None, None
    )
    ret_right, M2, d2, rvecs2, tvecs2 = cv2.calibrateCamera(
        objpoints, imgpoints_right, image_size, None, None
    )

    print(f"First monocular calibration completed:")
    print(f"  Left camera reprojection error: {ret_left:.4f}")
    print(f"  Right camera reprojection error: {ret_right:.4f}")

    # ============= 6. Compute per-frame monocular reprojection errors and remove outliers =============
    # Set a threshold, adjustable as needed, e.g., 1.0 or 1.5
    SINGLE_CAM_REPROJ_ERR_THRESH = 1.0

    errsL = []
    errsR = []
    for i, (rvec, tvec) in enumerate(zip(rvecs1, tvecs1)):
        projL, _ = cv2.projectPoints(objpoints[i], rvec, tvec, M1, d1)
        errL = cv2.norm(imgpoints_left[i], projL, cv2.NORM_L2) / len(projL)
        errsL.append(errL)

    for i, (rvec, tvec) in enumerate(zip(rvecs2, tvecs2)):
        projR, _ = cv2.projectPoints(objpoints[i], rvec, tvec, M2, d2)
        errR = cv2.norm(imgpoints_right[i], projR, cv2.NORM_L2) / len(projR)
        errsR.append(errR)

    # Filter according to threshold
    keep_indices = []
    for i in range(len(objpoints)):
        if errsL[i] < SINGLE_CAM_REPROJ_ERR_THRESH and errsR[i] < SINGLE_CAM_REPROJ_ERR_THRESH:
            keep_indices.append(i)

    num_all = len(objpoints)
    num_drop = num_all - len(keep_indices)
    if num_drop > 0:
        print(
            f"Removed {num_drop} outlier frames based on monocular reprojection error > {SINGLE_CAM_REPROJ_ERR_THRESH}")

    # Remaining corner points after filtering
    filtered_objpoints = [objpoints[i] for i in keep_indices]
    filtered_imgpoints_left = [imgpoints_left[i] for i in keep_indices]
    filtered_imgpoints_right = [imgpoints_right[i] for i in keep_indices]

    # ============= 7. Monocular calibration again (using filtered data) =============
    ret_left2, M1_2, d1_2, rvecs1_2, tvecs1_2 = cv2.calibrateCamera(
        filtered_objpoints, filtered_imgpoints_left, image_size, None, None
    )
    ret_right2, M2_2, d2_2, rvecs2_2, tvecs2_2 = cv2.calibrateCamera(
        filtered_objpoints, filtered_imgpoints_right, image_size, None, None
    )
    print(f"Monocular calibration after outlier removal completed:")
    print(f"  Left camera reprojection error: {ret_left2:.4f}")
    print(f"  Right camera reprojection error: {ret_right2:.4f}")

    # ============= 8. Stereo calibration (fixing intrinsics) =============
    # Let stereoCalibrate optimize only R, T, without modifying intrinsics
    criteria_stereo = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-6)
    flags = cv2.CALIB_FIX_INTRINSIC

    ret_stereo, M1_stereo, d1_stereo, M2_stereo, d2_stereo, R, T, E, F = cv2.stereoCalibrate(
        filtered_objpoints,
        filtered_imgpoints_left,
        filtered_imgpoints_right,
        M1_2, d1_2,  # Use second monocular calibration result as fixed starting point
        M2_2, d2_2,
        image_size,
        criteria=criteria_stereo,
        flags=flags
    )
    print(f"Stereo calibration completed, reprojection error: {ret_stereo:.4f}")

    baseline = np.linalg.norm(T)
    print("Translation vector T:\n", T)
    print(f"Baseline distance (meters): {baseline:.4f}")

    # ============= 9. Save final results into YAML (left.yaml, right.yaml) =============
    # Note: Saving M1_stereo/d1_stereo/M2_stereo/d2_stereo and R, T after stereoCalibrate, same as original format
    calib_dir = os.path.expanduser("~/stereo_calib_yaml_raw")
    if not os.path.isdir(calib_dir):
        os.makedirs(calib_dir)

    # Left camera YAML
    left_yaml_data = {
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_name": "stereo_left",
        "camera_matrix": {
            "rows": 3, "cols": 3,
            "data": M1_stereo.flatten().tolist()
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1, "cols": len(d1_stereo),
            "data": d1_stereo.flatten().tolist()
        },
        "R_stereo": {
            "rows": 3, "cols": 3,
            "data": R.flatten().tolist()
        },
        "T_stereo": {
            "rows": 3, "cols": 1,
            "data": T.flatten().tolist()
        }
    }

    # Right camera YAML
    right_yaml_data = {
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_name": "stereo_right",
        "camera_matrix": {
            "rows": 3, "cols": 3,
            "data": M2_stereo.flatten().tolist()
        },
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {
            "rows": 1, "cols": len(d2_stereo),
            "data": d2_stereo.flatten().tolist()
        },
        # Also write R, T into right camera YAML
        "R_stereo": {
            "rows": 3, "cols": 3,
            "data": R.flatten().tolist()
        },
        "T_stereo": {
            "rows": 3, "cols": 1,
            "data": T.flatten().tolist()
        }
    }

    left_yaml_file = os.path.join(calib_dir, "left.yaml")
    right_yaml_file = os.path.join(calib_dir, "right.yaml")
    with open(left_yaml_file, "w") as f:
        yaml.dump(left_yaml_data, f)
    with open(right_yaml_file, "w") as f:
        yaml.dump(right_yaml_data, f)

    print("Calibration results after outlier removal have been saved:")
    print(left_yaml_file)
    print(right_yaml_file)

if __name__ == '__main__':
    main()
