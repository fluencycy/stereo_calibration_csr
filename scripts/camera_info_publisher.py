#!/usr/bin/env python3
import rospy
import yaml
from sensor_msgs.msg import CameraInfo

# 初始化 ROS 节点
rospy.init_node('camera_info_publisher', anonymous=True)

# 获取参数文件路径和发布的话题名称
left_yaml_path = rospy.get_param('~left_yaml', 'left.yaml')
right_yaml_path = rospy.get_param('~right_yaml', 'right.yaml')
left_info_topic = rospy.get_param('~left_camera_info_topic', '/csr_full_test/left/camera_info')
right_info_topic = rospy.get_param('~right_camera_info_topic', '/csr_full_test/right/camera_info')

def load_camera_info(yaml_file):
    try:
        with open(yaml_file, "r") as file:
            calib_data = yaml.safe_load(file)
        rospy.loginfo("读取到的 YAML 文件 {} 内容：\n{}".format(yaml_file, calib_data))
        if calib_data is None:
            rospy.logfatal("YAML 文件 {} 内容为空。".format(yaml_file))
            rospy.signal_shutdown("YAML 文件为空")
        
        # 检查必须字段
        required_keys = ["image_width", "image_height", "distortion_model",
                         "distortion_coefficients", "camera_matrix",
                         "rectification_matrix", "projection_matrix"]
        for key in required_keys:
            if key not in calib_data:
                rospy.logfatal("YAML 文件 {} 缺少关键字段：{}".format(yaml_file, key))
                rospy.signal_shutdown("缺少关键字段")
        
        camera_info_msg = CameraInfo()
        camera_info_msg.width = int(calib_data["image_width"])
        camera_info_msg.height = int(calib_data["image_height"])
        camera_info_msg.distortion_model = calib_data["distortion_model"]

        # 将 data 中的数值转换为浮点数，防止读取后为字符串
        try:
            d_converted = [float(x) for x in calib_data["distortion_coefficients"]["data"]]
            k_converted = [float(x) for x in calib_data["camera_matrix"]["data"]]
            r_converted = [float(x) for x in calib_data["rectification_matrix"]["data"]]
            p_converted = [float(x) for x in calib_data["projection_matrix"]["data"]]
        except Exception as e:
            rospy.logfatal("转换 YAML 数据中的数值时出错：{}".format(e))
            rospy.signal_shutdown("数据转换错误")
        
        camera_info_msg.D = d_converted
        camera_info_msg.K = k_converted
        camera_info_msg.R = r_converted
        camera_info_msg.P = p_converted

        rospy.loginfo("转换后的 CameraInfo 参数：\n图像尺寸: {}x{}\nD: {}\nK: {}\nR: {}\nP: {}"
                      .format(camera_info_msg.width, camera_info_msg.height,
                              camera_info_msg.D, camera_info_msg.K,
                              camera_info_msg.R, camera_info_msg.P))
        return camera_info_msg
    except Exception as e:
        rospy.logfatal("加载 YAML 文件 {} 时出错：{}".format(yaml_file, e))
        rospy.signal_shutdown("加载 YAML 文件失败")
        return None

left_info = load_camera_info(left_yaml_path)
right_info = load_camera_info(right_yaml_path)

left_info_pub = rospy.Publisher(left_info_topic, CameraInfo, latch=True, queue_size=1)
right_info_pub = rospy.Publisher(right_info_topic, CameraInfo, latch=True, queue_size=1)

# 设置 frame_id（请根据需要调整，与图像消息的 frame_id 对应）
left_frame_id = rospy.get_param('~left_frame_id', 'left_camera')
right_frame_id = rospy.get_param('~right_frame_id', 'right_camera')
left_info.header.frame_id = left_frame_id
right_info.header.frame_id = right_frame_id

rospy.loginfo("开始发布 CameraInfo 参数...")
rate = rospy.Rate(1)  # 1 Hz 发布频率
topic_name = "/csr_full_test/left/camera_info"
while not rospy.is_shutdown():
    left_info_pub = rospy.Publisher(topic_name, CameraInfo, latch=True, queue_size=1)
    right_info_pub.publish(right_info)
    rospy.loginfo("已发布左相机和右相机的 CameraInfo 参数.")
    rate.sleep()
