#!/usr/bin/env python3
import cv2
import numpy as np
import glob
import os
import yaml

def compute_reprojection_error(objpoints, imgpoints, rvecs, tvecs, mtx, dist):
    total_error = 0
    total_points = 0
    per_image_errors = []
    for i in range(len(objpoints)):
        imgpoints2, _ = cv2.projectPoints(objpoints[i], rvecs[i], tvecs[i], mtx, dist)
        error = cv2.norm(imgpoints[i], imgpoints2, cv2.NORM_L2) ** 2
        n = len(objpoints[i])
        per_err = np.sqrt(error / n)
        per_image_errors.append(per_err)
        total_error += error
        total_points += n
    rms = np.sqrt(total_error / total_points)
    return rms, per_image_errors

def main():
    # 1. 设置棋盘格参数：内角点数和每个方格的真实尺寸（单位：米）
    pattern_size = (12, 9)  # 棋盘格内角点数
    square_size = 0.006     # 每个方格尺寸，单位：米
    criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)

    # 准备棋盘格在世界坐标下的三维点（所有图像都相同）
    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # 2. 载入采集的图像，存储所有图像的三维点与角点
    objpoints = []         # 3D 点（每一组相同）
    imgpoints_left = []    # 左相机角点
    imgpoints_right = []   # 右相机角点

    image_dir = os.path.expanduser("~/stereo_calib_images")
    left_images = sorted(glob.glob(os.path.join(image_dir, "left_*.png")))
    right_images = sorted(glob.glob(os.path.join(image_dir, "right_*.png")))

    if len(left_images) == 0 or len(right_images) == 0:
        print("未在目录 {} 中找到图像文件".format(image_dir))
        return

    for left_file, right_file in zip(left_images, right_images):
        img_left = cv2.imread(left_file)
        img_right = cv2.imread(right_file)
        gray_left = cv2.cvtColor(img_left, cv2.COLOR_BGR2GRAY)
        gray_right = cv2.cvtColor(img_right, cv2.COLOR_BGR2GRAY)

        ret_left, corners_left = cv2.findChessboardCorners(gray_left, pattern_size, None)
        ret_right, corners_right = cv2.findChessboardCorners(gray_right, pattern_size, None)
        if ret_left and ret_right:
            corners_left = cv2.cornerSubPix(gray_left, corners_left, (11, 11), (-1, -1), criteria)
            corners_right = cv2.cornerSubPix(gray_right, corners_right, (11, 11), (-1, -1), criteria)
            imgpoints_left.append(corners_left)
            imgpoints_right.append(corners_right)
            objpoints.append(objp)
            # 可选：显示检测结果
            cv2.drawChessboardCorners(img_left, pattern_size, corners_left, ret_left)
            cv2.drawChessboardCorners(img_right, pattern_size, corners_right, ret_right)
            cv2.imshow("Left Chessboard", img_left)
            cv2.imshow("Right Chessboard", img_right)
            cv2.waitKey(50)
        else:
            print("棋盘角点未检测到：", left_file, right_file)
    cv2.destroyAllWindows()

    # 假设所有图像尺寸一致，从第一幅图像获取尺寸
    image_size = gray_left.shape[::-1]

    # 3. 分别对左右相机进行单目标定
    ret_left, M1, d1, rvecs1, tvecs1 = cv2.calibrateCamera(objpoints, imgpoints_left, image_size, None, None)
    ret_right, M2, d2, rvecs2, tvecs2 = cv2.calibrateCamera(objpoints, imgpoints_right, image_size, None, None)
    print("单目标定结果：")
    print("左相机重投影误差: %.4f" % ret_left)
    print("右相机重投影误差: %.4f" % ret_right)

    # 4. 立体标定（初始标定）
    # 优化设置：使用单目标定结果作为初始猜测，固定主点，允许焦距和畸变系数优化
    flags = 0
    # flags |= cv2.CALIB_USE_INTRINSIC_GUESS
    # flags |= cv2.CALIB_FIX_PRINCIPAL_POINT
    # flags |= cv2.CALIB_SAME_FOCAL_LENGTH  # 如需强制左右焦距一致可启用
    criteria_stereo = (cv2.TERM_CRITERIA_MAX_ITER + cv2.TERM_CRITERIA_EPS, 100, 1e-6)

    ret_stereo, M1_stereo, d1_stereo, M2_stereo, d2_stereo, R, T, E, F = cv2.stereoCalibrate(
        objpoints, imgpoints_left, imgpoints_right,
        M1, d1, M2, d2, image_size,
        criteria=criteria_stereo, flags=flags
    )

    baseline = np.linalg.norm(T)
    print("初始立体标定结果：")
    print("立体标定重投影误差: %.4f" % ret_stereo)
    print("平移向量 T (单位: 米):\n", T)
    print("计算得到的基线距离: %.4f 米" % baseline)

    # 5. 计算每对图像的重投影误差，并剔除误差较大的图像对
    per_image_errors = []
    for i in range(len(objpoints)):
        # 分别用左右相机求解棋盘格姿态
        ret_l, rvec_l, tvec_l = cv2.solvePnP(objpoints[i], imgpoints_left[i], M1_stereo, d1_stereo, flags=cv2.SOLVEPNP_ITERATIVE)
        ret_r, rvec_r, tvec_r = cv2.solvePnP(objpoints[i], imgpoints_right[i], M2_stereo, d2_stereo, flags=cv2.SOLVEPNP_ITERATIVE)
        proj_left, _ = cv2.projectPoints(objpoints[i], rvec_l, tvec_l, M1_stereo, d1_stereo)
        proj_right, _ = cv2.projectPoints(objpoints[i], rvec_r, tvec_r, M2_stereo, d2_stereo)
        err_left = cv2.norm(imgpoints_left[i], proj_left, cv2.NORM_L2) / np.sqrt(len(objpoints[i]))
        err_right = cv2.norm(imgpoints_right[i], proj_right, cv2.NORM_L2) / np.sqrt(len(objpoints[i]))
        per_err = max(err_left, err_right)
        per_image_errors.append(per_err)
    
    mean_err = np.mean(per_image_errors)
    std_err = np.std(per_image_errors)
    threshold = mean_err + 2 * std_err
    print("图像对重投影误差均值: %.4f 像素, 标准差: %.4f" % (mean_err, std_err))
    print("剔除阈值设定为: %.4f 像素" % threshold)
    
    # 筛选出误差较大的图像对
    good_indices = [i for i, err in enumerate(per_image_errors) if err <= threshold]
    print("总共 %d 对图像中，剔除 %d 对异常图像" % (len(objpoints), len(objpoints) - len(good_indices)))
    
    # 若有剔除，则用过滤后的数据重新标定
    if len(good_indices) < len(objpoints):
        objpoints_filtered = [objpoints[i] for i in good_indices]
        imgpoints_left_filtered = [imgpoints_left[i] for i in good_indices]
        imgpoints_right_filtered = [imgpoints_right[i] for i in good_indices]
        ret_stereo, M1_stereo, d1_stereo, M2_stereo, d2_stereo, R, T, E, F = cv2.stereoCalibrate(
            objpoints_filtered, imgpoints_left_filtered, imgpoints_right_filtered,
            M1, d1, M2, d2, image_size,
            criteria=criteria_stereo, flags=flags
        )
        baseline = np.linalg.norm(T)
        print("过滤后立体标定结果：")
        print("立体标定重投影误差: %.4f" % ret_stereo)
        print("平移向量 T (单位: 米):\n", T)
        print("计算得到的基线距离: %.4f 米" % baseline)
    else:
        print("所有图像对均较好，无需剔除。")
    
    # 6. 立体校正，计算校正参数及 Q 矩阵
    R1_rect, R2_rect, P1_rect, P2_rect, Q, roi1, roi2 = cv2.stereoRectify(
        M1_stereo, d1_stereo, M2_stereo, d2_stereo, image_size, R, T,
        flags=cv2.CALIB_ZERO_DISPARITY, alpha=0
    )
    
    print("Q 矩阵:\n", Q)
    # 可根据 Q 矩阵验证基线： Q[3,2] 的倒数（绝对值）应接近实际基线
    if Q[3,2] != 0:
        baseline_from_Q = 1.0 / abs(Q[3,2])
        print("通过 Q 矩阵计算得到的基线: %.6f 米" % baseline_from_Q)
    else:
        print("Q 矩阵中基线信息异常。")
    
    # 7. 保存左右相机标定结果到 YAML 文件
    calib_left = {
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_name": "stereo_left",
        "camera_matrix": {"rows": 3, "cols": 3, "data": M1_stereo.flatten().tolist()},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": len(d1_stereo), "data": d1_stereo.flatten().tolist()},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": R1_rect.flatten().tolist()},
        "projection_matrix": {"rows": 3, "cols": 4, "data": P1_rect.flatten().tolist()}
    }
    calib_right = {
        "image_width": image_size[0],
        "image_height": image_size[1],
        "camera_name": "stereo_right",
        "camera_matrix": {"rows": 3, "cols": 3, "data": M2_stereo.flatten().tolist()},
        "distortion_model": "plumb_bob",
        "distortion_coefficients": {"rows": 1, "cols": len(d2_stereo), "data": d2_stereo.flatten().tolist()},
        "rectification_matrix": {"rows": 3, "cols": 3, "data": R2_rect.flatten().tolist()},
        "projection_matrix": {"rows": 3, "cols": 4, "data": P2_rect.flatten().tolist()}
    }
    
    calib_dir = os.path.expanduser("~/stereo_calib_yaml")
    if not os.path.isdir(calib_dir):
        os.makedirs(calib_dir)
    left_yaml = os.path.join(calib_dir, "left.yaml")
    right_yaml = os.path.join(calib_dir, "right.yaml")
    with open(left_yaml, "w") as f:
        yaml.dump(calib_left, f)
    with open(right_yaml, "w") as f:
        yaml.dump(calib_right, f)
    
    print("标定结果已保存至：")
    print(left_yaml)
    print(right_yaml)

if __name__ == '__main__':
    main()
