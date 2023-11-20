#!/usr/bin/env python3

import rospy
import tf
import geometry_msgs
import numpy as np
from nav_msgs.msg import Odometry
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import time

def callback(data):
    orient = rospy.Publisher('orientation', Vector3, queue_size=5)
    rate = rospy.Rate(10) # 10hz

    quat = data.orientation
    quat_list = [quat.x, quat.y, quat.z, quat.w]
    v = Vector3()
    roll, pitch, yaw = euler_from_quaternion(quat_list)
    v.x = roll*180/np.pi
    v.y = pitch*180/np.pi
    v.z = yaw*180/np.pi
    print()
    print("Roll:" + str(v.x))
    print("Pitch:" + str(v.y))
    print("Yaw:" + str(v.z))
    orient.publish(v)

def conversion():
    # Initilizes a default ros node 
    rospy.init_node('euler_conversion', anonymous=True)
    sub = rospy.Subscriber("imu/data", Imu, callback) # data recieved calls function callback()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()


if __name__ == '__main__':
    conversion()
