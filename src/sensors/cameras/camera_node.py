#!/usr/bin/env python

import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraNode:
    def __init__(self):
        rospy.init_node('camera_node', anonymous=True)
        self.image_pub = rospy.Publisher('camera/image', Image, queue_size=10)
        self.cap = cv2.VideoCapture(0)  # Assuming the camera is at index 0
        self.bridge = CvBridge()

    def capture_and_publish(self):
        rate = rospy.Rate(10)  # Adjust the rate based on your requirement

        while not rospy.is_shutdown():
            ret, frame = self.cap.read()
            if ret:
                image_msg = self.bridge.cv2_to_imgmsg(frame, encoding="bgr8")
                self.image_pub.publish(image_msg)

            rate.sleep()

if __name__ == '__main__':
    try:
        camera_node = CameraNode()
        camera_node.capture_and_publish()
    except rospy.ROSInterruptException:
        pass