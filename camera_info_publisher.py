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
# 加载标定参数（从 YAML 文件），确保 YAML 中包含必要字段
#############################################################################
def load_calibration(yaml_file):
    try:
        with open(yaml_file, "r") as file:
            calib_data = yaml.safe_load(file)
        rospy.loginfo("成功加载 YAML 文件：{}".format(yaml_file))
        required_keys = ["image_width", "image_height", "distortion_model",
                         "distortion_coefficients", "camera_matrix",
                         "rectification_matrix", "projection_matrix"]
        for key in required_keys:
            if key not in calib_data:
                rospy.logfatal("YAML 文件 {} 缺少关键字段：{}".format(yaml_file, key))
                rospy.signal_shutdown("缺少关键字段")
        calib = {}
        calib["image_width"] = int(calib_data["image_width"])
        calib["image_height"] = int(calib_data["image_height"])
        calib["D"] = np.array(calib_data["distortion_coefficients"]["data"], dtype=np.float32)
        calib["K"] = np.array(calib_data["camera_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        calib["R"] = np.array(calib_data["rectification_matrix"]["data"], dtype=np.float32).reshape(3, 3)
        calib["P"] = np.array(calib_data["projection_matrix"]["data"], dtype=np.float32).reshape(3, 4)
        return calib
    except Exception as e:
        rospy.logfatal("加载 YAML 文件 {} 失败：{}".format(yaml_file, e))
        rospy.signal_shutdown("加载 YAML 文件失败")
        return None

#############################################################################
# 计算棋盘格中所有横向相邻角点的距离平均值
# 输入：检测到的亚像素角点 corners_sub (shape: [N,1,2])
#        board_size: (num_cols, num_rows) 内角点数，如 (10,9)
# 返回：平均距离（单位与 3D 重投影一致，通常为米）
#############################################################################
def compute_average_square_length(corners_sub, board_size):
    num_cols, num_rows = board_size  # num_cols: 每行角点数；num_rows: 行数
    if corners_sub.shape[0] != num_cols * num_rows:
        rospy.logwarn("角点数量与预期不符！预期: {}，实际: {}".format(num_cols * num_rows, corners_sub.shape[0]))
        return None
    distances = []
    # 对每一行，计算相邻角点之间的欧氏距离
    for r in range(num_rows):
        for c in range(num_cols - 1):
            idx1 = r * num_cols + c
            idx2 = r * num_cols + c + 1
            pt1 = corners_sub[idx1][0]
            pt2 = corners_sub[idx2][0]
            d = np.linalg.norm(pt2 - pt1)
            if np.isfinite(d):
                distances.append(d)
    if len(distances) == 0:
        return None
    avg_distance = np.mean(distances)
    return avg_distance

#############################################################################
# stereo_callback: 接收左右图像，进行图像校正、立体匹配、3D 重投影，
# 并进行两处测量：棋盘格一整排方块尺寸平均值测量与图像中心深度测量
#############################################################################
def stereo_callback(left_img_msg, right_img_msg):
    try:
        left_img = bridge.imgmsg_to_cv2(left_img_msg, "bgr8")
        right_img = bridge.imgmsg_to_cv2(right_img_msg, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("图像转换错误: %s", e)
        return

    # 1. 图像校正
    rect_left = cv2.remap(left_img, left_map1, left_map2, cv2.INTER_LINEAR)
    rect_right = cv2.remap(right_img, right_map1, right_map2, cv2.INTER_LINEAR)

    # 2. 转为灰度图
    gray_left = cv2.cvtColor(rect_left, cv2.COLOR_BGR2GRAY)
    gray_right = cv2.cvtColor(rect_right, cv2.COLOR_BGR2GRAY)

    # 3. 计算左右视差（先计算左右视差，再利用 WLS 滤波）
    # 计算左视差（注意输出为 int16，值需要除以16转为实际视差）
    left_disp = stereo.compute(gray_left, gray_right).astype(np.int16)
    # 计算右视差
    right_disp = right_matcher.compute(gray_right, gray_left).astype(np.int16)
    # 使用 WLS 滤波器对左视差进行后处理
    filtered_disp = wls_filter.filter(left_disp, gray_left, None, right_disp)
    # 转换为 float32，并除以16得到实际视差
    filtered_disp = np.float32(filtered_disp) / 16.0
    # 将小于1.0的视差置0
    filtered_disp[filtered_disp < 1.0] = 0

    # 4. 利用滤波后的视差图重投影到3D空间（单位通常为米）
    points_3D = cv2.reprojectImageTo3D(filtered_disp, Q)

    # 5. 测量图像中心深度
    h, w = gray_left.shape
    center_x, center_y = w // 2, h // 2
    center_point = points_3D[center_y, center_x]
    center_depth = center_point[2]
    center_depth_mm = center_depth * 1000.0
    cv2.circle(rect_left, (center_x, center_y), 5, (0, 0, 255), -1)
    cv2.putText(rect_left, "Center Depth: {:.2f} mm".format(center_depth_mm),
                (center_x - 150, center_y - 20), cv2.FONT_HERSHEY_SIMPLEX,
                0.8, (0, 0, 255), 2)

    # # 6. 检测棋盘格并测量一整排方块的平均长度
    # # 假设棋盘内角点数为 (10,9)（即每行10个角点，代表9个方块）
    # board_pattern = (12, 9)
    # ret, corners = cv2.findChessboardCorners(gray_left, board_pattern,
    #                                           cv2.CALIB_CB_ADAPTIVE_THRESH + cv2.CALIB_CB_NORMALIZE_IMAGE)
    # square_length_avg = None
    # if ret:
    #     criteria_sub = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
    #     corners_sub = cv2.cornerSubPix(gray_left, corners, (11, 11), (-1, -1), criteria_sub)
    #     cv2.drawChessboardCorners(rect_left, board_pattern, corners_sub, ret)
    #     # 遍历每一行，计算相邻角点之间的 3D 距离（使用重投影结果）
    #     num_cols, num_rows = board_pattern
    #     distances = []
    #     for r in range(num_rows):
    #         for c in range(num_cols - 1):
    #             idx1 = r * num_cols + c
    #             idx2 = r * num_cols + c + 1
    #             pt1 = corners_sub[idx1][0]
    #             pt2 = corners_sub[idx2][0]
    #             x1, y1 = int(round(pt1[0])), int(round(pt1[1]))
    #             x2, y2 = int(round(pt2[0])), int(round(pt2[1]))
    #             if (0 <= x1 < points_3D.shape[1] and 0 <= y1 < points_3D.shape[0] and
    #                 0 <= x2 < points_3D.shape[1] and 0 <= y2 < points_3D.shape[0]):
    #                 P1 = points_3D[y1, x1]
    #                 P2 = points_3D[y2, x2]
    #                 d = np.linalg.norm(P2 - P1)
    #                 if np.isfinite(d):
    #                     distances.append(d)
    #     if len(distances) > 0:
    #         square_length_avg = np.mean(distances) * 1000.0  # 转换为毫米
    #         cv2.putText(rect_left, "Square Avg: {:.2f} mm".format(square_length_avg), (30, 60),
    #                     cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 255, 0), 2, cv2.LINE_AA)
    #         rospy.loginfo("测量到的平均方块边长: {:.2f} mm".format(square_length_avg))
    #     else:
    #         rospy.logwarn("未能获取有效的方块测量值。")
    # else:
    #     rospy.loginfo("未检测到棋盘格，请确保标定板在视野内。")

    # 7. 显示结果（可选：归一化后的视差图）
    disp_vis = cv2.normalize(filtered_disp, None, 0, 255, cv2.NORM_MINMAX)
    disp_vis = np.uint8(disp_vis)
    # cv2.imshow("Rectified Left", rect_left)
    # 如需显示右图和视差图，可解除下面注释
    # cv2.imshow("Rectified Right", rect_right)
    cv2.imshow("Disparity", disp_vis)
    cv2.waitKey(1)
    rospy.loginfo("Center depth: {:.2f} mm".format(center_depth_mm))

#############################################################################
# 主函数：加载标定参数、初始化映射表、构造 Q 矩阵、设置 StereoSGBM 和 WLS 滤波器，
# 并订阅左右图像
#############################################################################
if __name__ == "__main__":
    rospy.init_node("stereo_matching_with_measure_optimized", anonymous=True)
    bridge = CvBridge()

    # 从 ROS 参数中获取左右相机标定文件路径
    left_yaml_path = rospy.get_param('~left_yaml', '/home/cliu226/stereo_calib_yaml/left.yaml')
    right_yaml_path = rospy.get_param('~right_yaml', '/home/cliu226/stereo_calib_yaml/right.yaml')

    left_calib = load_calibration(left_yaml_path)
    right_calib = load_calibration(right_yaml_path)
    if left_calib is None or right_calib is None:
        rospy.logfatal("加载标定数据失败")
        exit(1)

    image_size = (left_calib["image_width"], left_calib["image_height"])
    rospy.loginfo("图像尺寸: {}".format(image_size))

    # 计算左右图像校正映射
    left_map1, left_map2 = cv2.initUndistortRectifyMap(
        left_calib["K"], left_calib["D"], left_calib["R"],
        left_calib["P"][:, :3], image_size, cv2.CV_16SC2)
    right_map1, right_map2 = cv2.initUndistortRectifyMap(
        right_calib["K"], right_calib["D"], right_calib["R"],
        right_calib["P"][:, :3], image_size, cv2.CV_16SC2)

    # 构造 Q 矩阵（用于从视差到 3D 重投影）
    fx = left_calib["K"][0, 0]
    cx = left_calib["K"][0, 2]
    cy = left_calib["K"][1, 2]
    Tx = -left_calib["P"][0, 3] / fx
    if abs(Tx) < 1e-6:
        Tx = -right_calib["P"][0, 3] / fx
    Q = np.array([[1, 0, 0, -cx],
                  [0, 1, 0, -cy],
                  [0, 0, 0,  fx],
                  [0, 0, -1/Tx, 0]], dtype=np.float32)
    rospy.loginfo("Q矩阵:\n{}".format(Q))

    # 设置 StereoSGBM 参数（请根据实际场景调整）
    blockSize = 1
    numDisparities = 128  # 必须为16的倍数
    stereo = cv2.StereoSGBM_create(
        minDisparity=0,
        numDisparities=numDisparities,
        blockSize=blockSize,
        P1=8 * 1 * blockSize**2,
        P2=32 * 1 * blockSize**2,
        disp12MaxDiff=1,
        preFilterCap=31,
        uniquenessRatio=10,
        speckleWindowSize=100,
        speckleRange=2
    )

    # 创建 WLS 滤波器，并配置参数
    wls_filter = ximgproc.createDisparityWLSFilter(matcher_left=stereo)
    # 参数 lambda: 越大平滑性越好，但会丢失细节；建议范围 8000～10000
    wls_filter.setLambda(10000)
    # 参数 sigmaColor: 控制边缘保留，建议 1.5～2.0
    wls_filter.setSigmaColor(1.5)
    # 创建右匹配器
    right_matcher = ximgproc.createRightMatcher(stereo)

    # 同步订阅左右图像话题，确保名称正确
    left_topic = rospy.get_param('~left_image_topic', '/csr_full_test/left/image_raw')
    right_topic = rospy.get_param('~right_image_topic', '/csr_full_test/right/image_raw')
    left_sub = Subscriber(left_topic, Image)
    right_sub = Subscriber(right_topic, Image)
    ats = ApproximateTimeSynchronizer([left_sub, right_sub], queue_size=5, slop=0.1)
    ats.registerCallback(stereo_callback)

    rospy.loginfo("stereo_matching_with_measure_optimized 节点启动，等待图像话题...")
    rospy.spin()
    cv2.destroyAllWindows()