# Description: Turns image topic from ROS to an opencv2 message and detects red.

import sys
import rospy
import cv2
import numpy as np
from std_msgs.msg import String
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError

class image_converter:

  def __init__(self):
    # self.image_pub = rospy.Publisher("image_topic_2",Image)

    self.bridge = CvBridge()
    self.image_sub = rospy.Subscriber("/camera3/usb_cam3/image_raw",Image,self.callback)

  def callback(self,data):
    try:
      imageFrame = self.bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      print(e)

    def calculate_distance(cx, cy, center_x, center_y):
      # Assuming a fixed real-world size for the object (adjust as needed)
      object_real_size_cm = 5.0  # Example: 10 cm

      # Calculate distance based on the size of the bounding box
      distance = object_real_size_cm * (0.35 / (w / imageFrame.shape[1]))  # Assumes width 'w' of bounding box represents real-world size
      return distance
    
    hsvFrame = cv2.cvtColor(imageFrame, cv2.COLOR_BGR2HSV)

    # Set range for red color and
    # define mask
    red_lower = np.array([136, 87, 111], np.uint8)
    red_upper = np.array([180, 255, 255], np.uint8)
    red_mask = cv2.inRange(hsvFrame, red_lower, red_upper)

    # Set range for green color and

    # Morphological Transform, Dilation
    # for each color and bitwise_and operator
    # between imageFrame and mask determines
    # to detect only that particular color
    kernel = np.ones((5, 5), "uint8")

    # For red color
    red_mask = cv2.dilate(red_mask, kernel)
    res_red = cv2.bitwise_and(imageFrame, imageFrame, mask=red_mask)

    # Creating contour to track red color
    contours, hierarchy = cv2.findContours(red_mask,
                                           cv2.RETR_TREE,
                                           cv2.CHAIN_APPROX_SIMPLE)

    largest_contour = None
    largest_contour_area = 0

    for pic, contour in enumerate(contours):
        area = cv2.contourArea(contour)
        if area > 300 and area > largest_contour_area:
            largest_contour = contour
            largest_contour_area = area

    if largest_contour is not None:
        x, y, w, h = cv2.boundingRect(largest_contour)
        imageFrame = cv2.rectangle(imageFrame, (x, y),
                                   (x + w, y + h),
                                   (0, 0, 255), 2)

        cv2.putText(imageFrame, "Red", (x, y),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0,
                    (0, 0, 255))

        # Calculate and display distance in centimeters
        distance = calculate_distance(x + w // 2, y + h // 2, imageFrame.shape[1] // 2, imageFrame.shape[0] // 2)
        cv2.putText(imageFrame, f'Distance: {distance:.2f} cm', (x, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

    imageFrame = cv2.flip(imageFrame,1)
    cv2.imshow("Image window", imageFrame)
    cv2.waitKey(3)

def main(args):
  ic = image_converter()
  rospy.init_node('image_converter', anonymous=True)
  try:
    rospy.spin()
  except KeyboardInterrupt:
    print("Shutting down")
  cv2.destroyAllWindows()

if __name__ == '__main__':
    main(sys.argv)