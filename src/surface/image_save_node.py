#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64, Header
from sensor_msgs.msg import Image, Joy
from geometry_msgs.msg import Vector3
import cv2
from cv_bridge import CvBridge, CvBridgeError
import os


bridge = CvBridge()
download_image: bool = False


def image_callback(image_data: Image):
    global download_image
    if download_image:
        rospy.loginfo("Downloading Image...")
        try:
            cv2_img = bridge.imgmsg_to_cv2(image_data, "bgr8")
        except CvBridgeError as e:
            rospy.logerr("Conversion to CV image failed")
            rospy.logerr(e)
        else:
            time = image_data.header.stamp
            folder = 'image_captures'

            if not os.path.exists(folder):
                rospy.loginfo(f'Creating directory {folder}')
                os.mkdir(folder)

            filename = folder + '/capture_' + str(time) + '.jpeg'

            if cv2.imwrite(filename, cv2_img):
                download_image = False
                rospy.loginfo(f"Image downloaded: '{filename}'")
            else:
                rospy.logerr("cv2.imwrite failed")


def controller_callback(data: Joy):
    global download_image
    if data.buttons[0] and not download_image:
        download_image = True
        rospy.loginfo("Queued image download")

    
def image_sub():
    rospy.init_node('image_save', anonymous=True)
    rospy.loginfo("Starting image save node")
    im_sub = rospy.Subscriber("/camera1/usb_cam1/image_raw", Image, image_callback, queue_size=1)
    joy_sub = rospy.Subscriber("joy", Joy, controller_callback)

    rospy.spin()


if __name__ == '__main__':
    image_sub()