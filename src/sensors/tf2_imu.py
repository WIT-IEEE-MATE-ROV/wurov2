#!/usr/bin/env python3

import rospy
import tf
import math

if __name__ == '__main__':
    rospy.init_node('dynamic_tf__imu_broadcaster')

    br = tf.TransformBroadcaster()
    rate = rospy.Rate(10.0)

    while not rospy.is_shutdown():
        t = rospy.Time.now().to_sec() * math.pi
        br.sendTransform((3.0 * math.sin(t), 3.0 * math.cos(t), 0.0),
                         (0.0, 0.0, 0.0, 1.0),
                         rospy.Time.now(),
                         "robot_2",
                         "world")
        rate.sleep()