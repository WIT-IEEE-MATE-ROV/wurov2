#!/usr/bin/env python3

import rospy
import time
import board
import math
import busio
from geometry_msgs.msg import Vector3, Quaternion
import numpy as np
import quaternion
# from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR
from adafruit_bno08x import (
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_ROTATION_VECTOR,
)

#BNO_REPORT_MAGNETOMETER,

from adafruit_bno08x.i2c import BNO08X_I2C
from sensor_msgs.msg import Imu
# import adafruit_bno055
from tf2_msgs.msg import TFMessage

LOOP_PERIOD_MS = 20.


def euler_from_quaternion(x, y, z, w):
    """
    Convert a quaternion into euler angles (roll, pitch, yaw)
    roll is rotation around x in radians (counterclockwise)
    pitch is rotation around y in radians (counterclockwise)
    yaw is rotation around z in radians (counterclockwise)
    """
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = math.atan2(t0, t1)
    
    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = math.asin(t2)
    
    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = math.atan2(t3, t4)
    
    return roll_x, pitch_y, yaw_z # in radians


def bno_main():
    imu_data_seq_counter = 0
    print('bno_node: Initializing...')
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    bno = BNO08X_I2C(i2c)
    # bno = adafruit_bno055.BNO055_I2C(i2c, address=0x28)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
    # bno.enable_feature(BNO_REPORT_ACCELEROMETER)
    # bno.enable_feature(BNO_REPORT_GYROSCOPE)
    # bno.enable_feature(BNO_REPORT_MAGNETOMETER)
    # bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)

    rospy.init_node('bno')
    pub_imu_data = rospy.Publisher('imu/data', Imu, queue_size=5)
    quat_publisher = rospy.Publisher('bno/quat', Quaternion, queue_size=3)
    euler_publisher = rospy.Publisher('bno/euler', Vector3, queue_size=3)

    time.sleep(0.5)

    rate = rospy.Rate(1/ (LOOP_PERIOD_MS / 1000))

    print('bno_node: Publishing')
    while not rospy.is_shutdown():
        quat = bno.game_quaternion         
        imu_data = Imu()  
        
        # quat = bno.quaternion
        # linear_acceleration = bno.acceleration
        # gyroscope = bno.gyro
        
        imu_data.header.stamp = rospy.Time.now()
        imu_data.header.frame_id = None
        imu_data.header.seq = imu_data_seq_counter

        imu_data.orientation.w = quat[0]
        imu_data.orientation.x = quat[1]
        imu_data.orientation.y = quat[2]
        imu_data.orientation.z = quat[3]

        # imu_data.linear_acceleration.x = linear_acceleration[0]
        # imu_data.linear_acceleration.y = linear_acceleration[1]
        # imu_data.linear_acceleration.z = linear_acceleration[2]

        # imu_data.angular_velocity.x = gyroscope[0]
        # imu_data.angular_velocity.y = gyroscope[1]
        # imu_data.angular_velocity.z = gyroscope[2]

        # imu_data.orientation_covariance[0] = -1
        # imu_data.linear_acceleration_covariance[0] = -1
        # imu_data.angular_velocity_covariance[0] = -1

        # imu_data_seq_counter=+1

        quat_ros = Quaternion(quat[0], quat[1], quat[2], quat[3])
        euler = [c * 180/3.14159 for c in euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])]
        euler_ros = Vector3(euler[0], euler[1], euler[2])

# x: -0.1226560957111919
# y: -0.6980003645098898
# z: 0.704172011831037
# w: 0.04294258101077972
        zero = np.quaternion(0.04294258101077972, -0.1226560957111919, -0.6980003645098898, 0.704172011831037)
        q = np.quaternion(quat_ros.w, quat_ros.x, -quat_ros.y, quat_ros.z)
        # q = q * np.quaternion(-math.sqrt(2) / 2, 0, -math.sqrt(2) / 2, 0)
        # q = q * np.quaternion(math.sqrt(2) / 2, math.sqrt(2) / 2, 0, 0)
        # q = q * np.quaternion(0, math.sqrt(2) / 2, math.sqrt(2) / 2, 0)
        quat_ros.w = q.w
        quat_ros.x = q.x
        quat_ros.y = q.y
        quat_ros.z = q.z

        e = euler_from_quaternion(q.x, q.y, q.z, q.w)
        euler_ros.x = e[0] * 180 / math.pi
        euler_ros.y = e[1] * 180 / math.pi
        euler_ros.z = e[2] * 180 / math.pi
        pub_imu_data.publish(imu_data)
        quat_publisher.publish(quat_ros)
        euler_publisher.publish(euler_ros)

#        print(f'euler: {euler}\nquat: {quat}')
        
        rate.sleep()

    rospy.spin()

    
if __name__ == '__main__':
    bno_main()
