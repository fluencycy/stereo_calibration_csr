#!/usr/bin/env python3
import rospy
import cv2
import cv2.ximgproc as ximgproc
import numpy as np
import yaml
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from message_filters import Subscriber, ApproximateTimeSynchronizer

#############################################################################
# Load calibration parameters (from YAML file), required fields only:
#  - image_width, image_height, distortion_model
#  - distortion_coefficients, camera_matrix
#  - R_stereo, T_stereo
#############################################################################
def load_calibration(yaml_file, camera_name=""):
    with open(yaml_file, "r") as file:
        calib_data = yaml.safe_load(file)
    rospy.loginfo("Successfully loaded YAML file: {} -> {}".format(yaml_file, camera_name))

    required_keys = [
        "image_width",
        "image_height",
        "distortion_model",
        "distortion_coefficients",
        "camera_matrix",
        "R_stereo",
        "T_stereo"
    ]
    for key in required_keys:
        if key not in calib_data:
            rospy.logfatal("YAML file {} is missing key field: {}".format(yaml_file, key))
            rospy.signal_shutdown("Missing key field")
            return None

    calib = {}
    calib["image_width"] = int(calib_data["image_width"])
    calib["image_height"] = int(calib_data["image_height"])
    calib["distortion_model"] = calib_data["distortion_model"]

    # Distortion coefficients
    dist_coeffs = calib_data["distortion_coefficients"]["data"]
    calib["D"] = np.array(dist_coeffs, dtype=np.float64)

    # Camera matrix
    cam_matrix = calib_data["camera_matrix"]["data"]
    calib["K"] = np.array(cam_matrix, dtype=np.float64).reshape(3, 3)

    # Stereo extrinsics: R_stereo, T_stereo
    R_st = calib_data["R_stereo"]["data"]
    T_st = calib_data["T_stereo"]["data"]
    # Note the reshape
    calib["R_stereo"] = np.array(R_st, dtype=np.float64).reshape(3, 3)
    calib["T_stereo"] = np.array(T_st, dtype=np.float64).reshape(3, 1)

    return calib

#############################################################################
# Configure checkerboard specification (inner corners), used for real-time measuring
#############################################################################
pattern_size = (11, 9)  # (columns, rows), example: 9x6
criteria_subpix = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

#############################################################################
# stereo_callback: perform image rectification, stereo matching, re-projection,
# and display depth + checkerboard measurement
#############################################################################
def stereo_callback(left_img_msg, right_img_msg):
    try:
        left_img = bridge.imgmsg_to_cv2(left_img_msg, "bgr8")
        right_img = bridge.imgmsg_to_cv2(right_img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Image conversion error: %s", e)
        return

    # 1. Rectification (remap)
    rect_left = cv2.remap(left_img, left_map1, left_map2, cv2.INTER_LINEAR)
    rect_right = cv2.remap(right_img, right_map1, right_map2, cv2.INTER_LINEAR)

    # 2. Convert to grayscale
    gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

    # 3. Compute disparity (SGBM + WLS)
    left_disp = stereo.compute(gray_left, gray_right).astype(np.int16)
    right_disp = right_matcher.compute(gray_right, gray_left).astype(np.int16)
    filtered_disp = wls_filter.filter(left_disp, gray_left, None, right_disp)
    filtered_disp = np.float32(filtered_disp) / 16.0
    filtered_disp[filtered_disp < 1.0] = 0  # Set values less than 1 to 0 (invalid)

    # 4. Reproject disparity map to 3D
    points_3D = cv2.reprojectImageTo3D(filtered_disp, Q)  # Unit: meters (if T is in meters)

    # 5. Measure center depth (demo)
    h, w = gray_left.shape
    cx, cy = w // 2, h // 2
    center_3D = points_3D[cy, cx]  # (X, Y, Z)
    center_depth_m = center_3D[2]
    center_depth_mm = center_depth_m * 1000.0
    cv2.circle(rect_left, (cx, cy), 5, (0, 0, 255), -1)
    cv2.putText(rect_left, "Center Depth: {:.2f} mm".format(center_depth_mm),
                (cx - 150, cy - 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 0, 255), 2)

    ############################################################################
    # 6. (New) Detect checkerboard corners and calculate length of all squares
    #    - Find corners on rect_left
    #    - Project to 3D for coordinates
    #    - Compute distance between adjacent corners, output matrix & average
    ############################################################################
    ret, corners2D = cv2.findChessboardCorners(gray_left, pattern_size,
                                               flags=cv2.CALIB_CB_ADAPTIVE_THRESH +
                                                     cv2.CALIB_CB_NORMALIZE_IMAGE +
                                                     cv2.CALIB_CB_FAST_CHECK)
    if ret:
        # Refine to subpixel accuracy
        corners2D = cv2.cornerSubPix(gray_left, corners2D, (11, 11), (-1, -1), criteria_subpix)

        # Convert 2D corners to 3D
        corners2D = corners2D.reshape(-1, 2)
        corner_points_3d = []
        for pt in corners2D:
            x2d, y2d = pt
            x2d_int = int(round(x2d))
            y2d_int = int(round(y2d))
            # Check if within image boundaries
            if 0 <= x2d_int < w and 0 <= y2d_int < h:
                p3d = points_3D[y2d_int, x2d_int]  # (X, Y, Z)
                corner_points_3d.append(p3d)
            else:
                corner_points_3d.append([0, 0, 0])

        corner_points_3d = np.array(corner_points_3d).reshape(
            (pattern_size[1], pattern_size[0], 3)
        )  # Shape: (rows, columns, 3)

        # Calculate horizontal and vertical distances between adjacent corners
        horiz_dist_mat = np.zeros((pattern_size[1], pattern_size[0]-1), dtype=np.float32)
        vert_dist_mat = np.zeros((pattern_size[1]-1, pattern_size[0]), dtype=np.float32)

        # Horizontal distances
        for row in range(pattern_size[1]):
            for col in range(pattern_size[0] - 1):
                p1 = corner_points_3d[row, col]
                p2 = corner_points_3d[row, col+1]
                dist = np.linalg.norm(p2 - p1) if p1[2] > 0.0001 and p2[2] > 0.0001 else 0.0
                horiz_dist_mat[row, col] = dist

        # Vertical distances
        for row in range(pattern_size[1] - 1):
            for col in range(pattern_size[0]):
                p1 = corner_points_3d[row, col]
                p2 = corner_points_3d[row+1, col]
                dist = np.linalg.norm(p2 - p1) if p1[2] > 0.0001 and p2[2] > 0.0001 else 0.0
                vert_dist_mat[row, col] = dist

        # Print matrices
        rospy.loginfo("Horizontal length matrix (m):\n{}".format(horiz_dist_mat))
        rospy.loginfo("Vertical length matrix (m):\n{}".format(vert_dist_mat))

        # Calculate average length (excluding zeros)
        horiz_nonzero = horiz_dist_mat[horiz_dist_mat > 0.0]
        vert_nonzero = vert_dist_mat[vert_dist_mat > 0.0]
        if len(horiz_nonzero) + len(vert_nonzero) > 0:
            avg_len_m = np.mean(np.concatenate((horiz_nonzero, vert_nonzero)))
            avg_len_mm = avg_len_m * 1000.0
            rospy.loginfo("Average length of the checkerboard: {:.3f} mm".format(avg_len_mm))

        # Visualize corners on rect_left
        cv2.drawChessboardCorners(rect_left, pattern_size, corners2D.reshape(-1, 1, 2), True)
    # 7. Display disparity and left rectified image (with corners)
    disp_vis = cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = np.uint8(disp_vis)
    cv2.imshow("Rectified Left + Chessboard", rect_left)
    cv2.imshow("Disparity", disp_vis)
    cv2.waitKey(1)

    # Log center depth (for demonstration)
    rospy.loginfo("Depth at the center: {:.2f} mm".format(center_depth_mm))


#############################################################################
# Main function: load calibration files, perform stereoRectify, and subscribe to stereo images
#############################################################################
if __name__ == "__main__":
    rospy.init_node("stereo_matching_node", anonymous=True)
    bridge = CvBridge()

    # 1. Load left and right calibration YAML (containing only K, D, R_stereo, T_stereo)
    left_yaml_path = rospy.get_param('~left_yaml', '/home/cliu226/stereo_calib_yaml_raw/left.yaml')
    right_yaml_path = rospy.get_param('~right_yaml', '/home/cliu226/stereo_calib_yaml_raw/right.yaml')

    left_calib = load_calibration(left_yaml_path, camera_name="LeftCamera")
    right_calib = load_calibration(right_yaml_path, camera_name="RightCamera")
    if left_calib is None or right_calib is None:
        rospy.logfatal("Failed to load calibration data")
        exit(1)

    width = left_calib["image_width"]
    height = left_calib["image_height"]
    K_left = left_calib["K"]
    D_left = left_calib["D"]
    K_right = right_calib["K"]
    D_right = right_calib["D"]
    R_stereo = left_calib["R_stereo"]  # Left to Right
    T_stereo = left_calib["T_stereo"]  # Left to Right

    # If T_stereo is in meters, no need to convert; if in mm during calibration, uncomment next line:
    # T_stereo /= 1000.0  # Uncomment if unit was mm during calibration

    image_size = (width, height)
    rospy.loginfo("Image size: {}".format(image_size))

    # 2. Perform stereoRectify
    R1, R2, P1, P2, Q, roi1, roi2 = cv2.stereoRectify(
        cameraMatrix1=K_left,
        distCoeffs1=D_left,
        cameraMatrix2=K_right,
        distCoeffs2=D_right,
        imageSize=image_size,
        R=R_stereo,
        T=T_stereo,
        flags=cv2.CALIB_ZERO_DISPARITY,
        alpha=0
    )
    rospy.loginfo("stereoRectify done, Q matrix:\n{}".format(Q))

    # 3. initUndistortRectifyMap
    left_map1, left_map2 = cv2.initUndistortRectifyMap(
        K_left, D_left, R1, P1, image_size, cv2.CV_16SC2
    )
    right_map1, right_map2 = cv2.initUndistortRectifyMap(
        K_right, D_right, R2, P2, image_size, cv2.CV_16SC2
    )

    # 4. Configure stereo matching (SGBM + WLS), tune parameters as needed for your scene
    blockSize = 10
    numDisparities = 64
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=numDisparities,
        blockSize=blockSize,
        P1=8 * blockSize**2,
        P2=32 * blockSize**2,
        disp12MaxDiff=1,
        preFilterCap=31,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2
    )
    wls_filter = ximgproc.createDisparityWLSFilter(matcher_left=stereo)
    # You can fine-tune lambda and sigmaColor based on your scene
    wls_filter.setLambda(30000)
    wls_filter.setSigmaColor(1)
    right_matcher = ximgproc.createRightMatcher(stereo)

    # 5. Subscribe to left and right image topics with synchronization
    left_topic = rospy.get_param('~left_image_topic', '/csr_full_test/left/image_raw')
    right_topic = rospy.get_param('~right_image_topic', '/csr_full_test/right/image_raw')
    left_sub = Subscriber(left_topic, Image)
    right_sub = Subscriber(right_topic, Image)
    ats = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=5, slop=0.1)
    ats.registerCallback(stereo_callback)

    rospy.loginfo("Stereo matching node started, waiting for stereo image streams ...")
    rospy.spin()
    cv2.destroyAllWindows()
