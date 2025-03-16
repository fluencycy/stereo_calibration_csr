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
        # Subscribe to left and right image topics
        self.left_sub = self.ral.subscriber(self.left_topic, Image, self.left_callback)
        self.right_sub = self.ral.subscriber(self.right_topic, Image, self.right_callback)

    def left_callback(self, msg):
        try:
            self.left_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Left image conversion error: %s", e)

    def right_callback(self, msg):
        try:
            self.right_frame = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Right image conversion error: %s", e)

    def wait_until_first_frame(self, timeout=10):
        rospy.loginfo("Waiting for the first image frame...")
        start = time.time()
        while self.left_frame is None or self.right_frame is None:
            if time.time() - start > timeout:
                raise Exception("Timeout while waiting for left and right images")
            rospy.sleep(0.1)
        rospy.loginfo("First image frame received successfully.")


def main():
    rospy.init_node("stereo_image_capture_manual", anonymous=True)

    # Get left and right image topics from parameter server
    left_topic = rospy.get_param('~left_image_topic', '/csr_full_test/left/image_raw')
    right_topic = rospy.get_param('~right_image_topic', '/csr_full_test/right/image_raw')

    # Create crtk.ral object
    ral = crtk.ral("stereo_image_capture_manual")
    subscriber = StereoImageSubscriber(ral, left_topic, right_topic)
    subscriber.wait_until_first_frame()

    # Create left image display window for monitoring current frame
    cv2.namedWindow("Left Image", cv2.WINDOW_NORMAL)
    cv2.resizeWindow("Left Image", 640, 480)

    # Directory for saving images
    output_dir = os.path.expanduser("~/stereo_calib_images")
    if not os.path.isdir(output_dir):
        os.makedirs(output_dir)
    count = 0

    rospy.loginfo("Stereo Image Capture (Manual) started.")
    rospy.loginfo("Press 'Enter' to capture and save stereo image pair, press 'q' to quit.")

    while not rospy.is_shutdown():
        if subscriber.left_frame is not None:
            # Display left image
            display_img = subscriber.left_frame.copy()
            cv2.imshow("Left Image", display_img)

        key = cv2.waitKey(1) & 0xFF
        if key == ord('q'):
            rospy.loginfo("Exit command detected, terminating program.")
            break
        elif key == 13:  # ASCII code for Enter key is usually 13
            if subscriber.left_frame is not None and subscriber.right_frame is not None:
                left_path = os.path.join(output_dir, "left_%d.png" % count)
                right_path = os.path.join(output_dir, "right_%d.png" % count)
                cv2.imwrite(left_path, subscriber.left_frame)
                cv2.imwrite(right_path, subscriber.right_frame)
                rospy.loginfo("Saved image pair %d: %s, %s", count, left_path, right_path)
                count += 1
            else:
                rospy.logwarn("No complete left and right images received yet.")

    cv2.destroyAllWindows()


if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass
