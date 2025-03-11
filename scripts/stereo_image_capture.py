#!/usr/bin/env python3
import rospy
import cv2
import os
import time
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
import crtk

class StereoImageSubscriber:
    def __init__(self, ral, left_topic, right_topic):
        self.ral = ral
        self.left_topic = left_topic
        self.right_topic = right_topic
        self.bridge = CvBridge()
        self.left_frame = None
        self.right_frame = None
        # 订阅左右图像
        self.left_sub = self.ral.subscriber(self.left_topic, Image, self.left_callback)
        self.right_sub = self.ral.subscriber(self.right_topic, Image, self.right_callback)

    def left_callback(self, msg):
        try:
            self.left_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("左图像转换错误: %s", e)

    def right_callback(self, msg):
        try:
            self.right_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("右图像转换错误: %s", e)

    def wait_until_first_frame(self, timeout=10):
        rospy.loginfo("等待第一帧图像...")
        start = time.time()
        while self.left_frame is None or self.right_frame is None:
            if time.time() - start > timeout:
                raise Exception("等待左右图像超时")
            rospy.sleep(0.1)
        rospy.loginfo("第一帧图像接收成功。")

def main():
    rospy.init_node("stereo_image_capture_manual", anonymous=True)
    
    # 从参数服务器获取左右图像话题
    left_topic = rospy.get_param('~left_image_topic', '/csr_full_test/left/image_raw')
    right_topic = rospy.get_param('~right_image_topic', '/csr_full_test/right/image_raw')
    
    # 创建 crtk.ral 对象
    ral = crtk.ral("stereo_image_capture_manual")
    subscriber = StereoImageSubscriber(ral, left_topic, right_topic)
    subscriber.wait_until_first_frame()

    # 创建左侧图像显示窗口，用于观察当前画面
    cv2.namedWindow("Left Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Left Image", 640, 480)

    # 图像保存目录
    output_dir = os.path.expanduser("~/stereo_calib_images")
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    count = 0

    rospy.loginfo("Stereo Image Capture (Manual) 已启动。")
    rospy.loginfo("按 'Enter' 键捕获并保存左右图像对，按 'q' 键退出。")

    while not rospy.is_shutdown():
        if subscriber.left_frame is not None:
            # 显示左侧图像
            display_img = subscriber.left_frame.copy()
            cv2.imshow("Left Image", display_img)
        
        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.loginfo("检测到退出指令，程序终止。")
            break
        elif key == 13:  # Enter 键的ASCII码通常为13
            if subscriber.left_frame is not None and subscriber.right_frame is not None:
                left_path = os.path.join(output_dir, "left_%d.png" % count)
                right_path = os.path.join(output_dir, "right_%d.png" % count)
                cv2.imwrite(left_path, subscriber.left_frame)
                cv2.imwrite(right_path, subscriber.right_frame)
                rospy.loginfo("已保存图像对 %d：%s, %s", count, left_path, right_path)
                count += 1
            else:
                rospy.logwarn("当前未接收到完整的左右图像。")
    
    cv2.destroyAllWindows()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
