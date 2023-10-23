#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

def callback(data):
    xyz = rospy.Publisher('pca_linearMovement', Vector3, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    v = Vector3()
    v.y = data.axes[1]
    v.x = data.axes[0]
    xyz.publish(v)
    
    
def joystick_pub():

    # In ROS, nodes are uniquely named. If two nodes with the same
    # name are launched, the previous one is kicked off. The
    # anonymous=True flag means that rospy will choose a unique
    # name for our 'listener' node so that multiple listeners can
    # run simultaneously.
    rospy.init_node('joystick', anonymous=True)

    sub = rospy.Subscriber("joy", Joy, callback)

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    joystick_pub()