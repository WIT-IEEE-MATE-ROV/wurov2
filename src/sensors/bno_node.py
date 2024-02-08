#!/usr/bin/env python3

import rospy
import time
import board
import math
import busio
from geometry_msgs.msg import Vector3, Quaternion, Pose, Point, TransformStamped, Transform
from std_msgs.msg import Header
from adafruit_bno08x import BNO_REPORT_GAME_ROTATION_VECTOR
from adafruit_bno08x.i2c import BNO08X_I2C
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
    print('bno_node: Initializing...')
    i2c = busio.I2C(board.SCL, board.SDA, frequency=100000)
    bno = BNO08X_I2C(i2c)
    bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)

    rospy.init_node('bno')
    quat_publisher = rospy.Publisher('bno/quat', Quaternion, queue_size=3)
    euler_publisher = rospy.Publisher('bno/euler', Vector3, queue_size=3)
    pose_publisher = rospy.Publisher('bno/pose', TFMessage, queue_size=3)

    sequence = 0
    timestamp = rospy.Time().now()
    frame_id = 'bno_pose'
    header = Header(sequence, timestamp, frame_id)

    translation = Vector3(0, 0, 0)
    

    current_transform = Transform(translation, Quaternion(0, 0, 0, 0))
    current_transform_stamped = TransformStamped(header, 'poses', current_transform)

    tf_message = TFMessage([current_transform_stamped])

    time.sleep(0.5)

    rate = rospy.Rate(1/ (LOOP_PERIOD_MS / 1000))

    print('bno_node: Publishing')
    while not rospy.is_shutdown():
        quat = bno.game_quaternion
        quat_ros = Quaternion(quat[0], quat[1], quat[2], quat[3])

        euler = [c * 180/3.14159 for c in euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])]
        euler_ros = Vector3(euler[0], euler[1], euler[2])

        pose = Pose(Point(0,0,0), quat_ros)

        current_transform = Transform(translation, quat_ros)

        header.seq = sequence
        header.stamp = rospy.Time().now()

        # current_transform_stamped.header = header
        # current_transform_stamped.transform = current_transform

        tf_message.transforms[0].header = header
        tf_message.transforms[0].transform = current_transform


        quat_publisher.publish(quat_ros)
        euler_publisher.publish(euler_ros)
        pose_publisher.publish(tf_message)

#        print(f'euler: {euler}\nquat: {quat}')
        
        sequence += 1
        rate.sleep()

    rospy.spin()

    
if __name__ == '__main__':
    bno_main()
