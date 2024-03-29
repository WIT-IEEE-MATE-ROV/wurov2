#!/usr/bin/env python3

import math

import cv2
import numpy as np
from scipy import ndimage
from cameras import Camera
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image, CompressedImage

dim = (640, 480)

bridge = CvBridge()

def detect_red_square(frame):

    # Read the image    
    # img = ndimage.rotate(img, 45)
    img_blur = cv2.GaussianBlur(frame, (15, 15), 0)

    # Convert the image from BGR to HSV
    img_hsv = cv2.cvtColor(img_blur, cv2.COLOR_BGR2HSV)
    # img_hsv_blur = cv2.GaussianBlur(img_hsv, (7, 7), 0)
    img_hsv_blur = img_hsv

    # cv2.imshow('Blurred', cv2.cvtColor(cv2.resize(img_hsv_blur, dim), cv2.COLOR_HSV2BGR))
    # cv2.waitKey(0)
    # Define the lower and upper bounds for red color in HSV
    # lower_red = np.array([0, 197, 121])
    # upper_red = np.array([179, 255, 255])
    #(0, 109, 26), (7, 255, 255)
    # lower_red, upper_red = (131, 80, 95), (179, 237, 255)
    lower_red, upper_red = (0, 63, 60), (23, 255, 189)

    # Create a mask to extract only red pixels
    mask = cv2.inRange(img_hsv_blur, lower_red, upper_red)
    
    # cv2.imshow('Mask', mask)
    # cv2.waitKey(0)

    # Find contours in the mask
    contours, hierarchy = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_TC89_KCOS)
    if len(contours) == 0:
        print('No contours found')
    # Loop through the contours
    for contour in contours:
        # print('Contour:', contour)
        # Approximate the contour to a polygon
        epsilon = 0.02 * cv2.arcLength(contour, True)
        approx = cv2.approxPolyDP(contour, epsilon, True)

        # If the polygon has 4 corners, it's likely a square
        # if len(approx) == 4:
        # Draw a rectangle around the square
        cv2.drawContours(frame, [approx], 0, (0, 255, 0), 1)

    try:
        hierarchy = hierarchy[0]
    except:
        hierarchy = []


    # computes the bounding box for the contour, and draws it on the frame,
    for contour, hier in zip(contours, hierarchy):
        (x, y, w, h) = cv2.boundingRect(contour)
        if 80 < w < 1000 and 40 < h < 1000:
            # cv2.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 2)

            hull = cv2.minAreaRect(contour)
            bp = cv2.boxPoints(hull)
            # print(bp)
            box_width = 3.58
            pixels_per_inch = (bp[2][0] - bp[1][0]) / box_width
            est_height = (bp[3][1] - bp[2][1]) / pixels_per_inch
            print(f'pixels_per_inch = {pixels_per_inch}')
            print('Estimated Height = %.02fin' % (est_height))
            # cv2.putText(img, '%.02fin' % est_height)
            box = np.intp(bp)
            frame = cv2.drawContours(frame, [box], 0, (0, 0, 255), 2)
    
    return frame

    # Display the result
    # cv2.imshow('Red Square Detection', cv2.resize(img, (1920,1080)))
    # cv2.imshow('Red Square Detection', frame)

    # cv2.waitKey(0)
    # cv2.destroyAllWindows()


# Example usage
def image_callback(image_data: Image, result_publisher: rospy.Publisher):
    rospy.loginfo("Detecting Square")
    try:
        cv2_img = bridge.imgmsg_to_cv2(image_data, "bgr8")
    except CvBridgeError as e:
        rospy.logerr("Conversion to CV image failed")
        rospy.logerr(e)
    else:
        frame = detect_red_square(cv2_img)
        msg = CompressedImage()
        msg.header.stamp = rospy.Time.now()
        msg.format = "jpeg"
        msg.data = np.array(cv2.imencode('.jpg', frame)[1]).tostring()
        result_publisher.publish(msg)



def detect_pub():
    # cam = Camera(1080, 720, "/dev/video0").start()
    rospy.init_node('square_detect', anonymous=True)
    detect_pub = rospy.Publisher('/square_detect/compressed', CompressedImage)
    im_sub = rospy.Subscriber("/camera1/usb_cam1/image_raw", Image, image_callback, (detect_pub), queue_size=1)

    rospy.spin()

    # while not rospy.is_shutdown():
    #     _, f = cam.read()
        
    #     frame = detect_red_square(f)
    #     cam.view(frame)


# TODO: Argument for camera selection
if __name__ == '__main__':
    detect_pub()

