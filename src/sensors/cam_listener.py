#!/usr/bin/env python


import rospy
from rospy_tutorials.msg import Floats

def callback(data):
    print (rospy.get_name(), "I heard %s"%str(data.data))

def listener():
    rospy.init_node('listener')
    print("listener started")
    rospy.Subscriber("cam_data", Floats, callback)
    rospy.spin()

if __name__ == '__main__':
    listener()
