import numpy as np
import cv2 as cv
import os
import rospy
from rospy.numpy_msg import numpy_msg
from rospy_tutorials.msg import Floats

"""

ros Publisher that gets frames from a video
Hopefully ML and AI applications will sub to this Publisher to process the video data

"""


def get_vide_frames():
    ## found this here -> http://wiki.ros.org/rospy_tutorials/Tutorials/numpy
    # not sending ndarray
    pub = rospy.Publisher('cam_data', numpy_msg(Floats), queue_size=10)
    rospy.init_node("camera")
    rate = rospy.Rate(10)
    ##

    # TODO - add more cameras and check if operational
    vcap = cv.VideoCapture(0)
    if not vcap.isOpened():
        print("The camera cant be opened")
        exit()

    try:
        while vcap.isOpened():
            # if rospy.is_shutdown() == True:
            #     print("Ros is shutdown")
            #     break

            ret, frame = vcap.read()
            print(frame)
            if not ret:
                print("[ERROR] cant get frames")
                break
        
            pub.publish(frame)
            rate.sleep()
            cv.imshow("fames", frame)
            #TODO - have a more 'automatic' shutdown
            if cv.waitKey(1) == ord('q'):
                break
    except KeyboardInterrupt:
        print("\nProgramed exit\n")


if __name__ == "__main__":
    get_vide_frames()
