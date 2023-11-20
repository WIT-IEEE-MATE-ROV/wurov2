#!/usr/bin/env python3

# By: Tedi Qafko
# Date: November 19, 2023

# Description:
# Retrives filtered data from the imu madwick filter and transforms
# quartenions to euler angles. The angles are than published to a topic.


# Libraries
import rospy
import tf
import geometry_msgs
import numpy as np
import time
from tf.transformations import euler_from_quaternion, quaternion_from_euler
from nav_msgs.msg import Odometry
from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

def callback(data):
    # Publishes roll, pitch, and yaw values after data is recived from the subscriber
    roll_pitch_yaw = rospy.Publisher('orientation', Vector3, queue_size=5)
    rate = rospy.Rate(10) # 10hz
    
    v = Vector3()   # A vector x, y, z msg variable
    quat = data.orientation # Retrives quartenions from subscriber data
    quat_list = [quat.x, quat.y, quat.z, quat.w]
    roll, pitch, yaw = euler_from_quaternion(quat_list) # Using TF, transforms quaternions to euler

    # Roll, Pitch, Yaw saved in vector variable
    # Convert: Radians to Degrees
    v.x =  roll * 180/np.pi
    v.y = pitch * 180/np.pi
    v.z =   yaw * 180/np.pi

    # print()
    # print("Roll:" + str(v.x))
    # print("Pitch:" + str(v.y))
    # print("Yaw:" + str(v.z))

    # Publish data to topic
    roll_pitch_yaw.publish(v)

def conversion():
    # Waits for data to be retrived from the imu madwick filter and calls for conversion
    rospy.init_node('euler_conversion', anonymous=True)
    sub = rospy.Subscriber("imu/data", Imu, callback) # data recieved calls function callback()
    rospy.spin()

if __name__ == '__main__':
    conversion()
