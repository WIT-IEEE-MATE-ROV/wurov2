#!/usr/bin/env python
import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3

def callback(data):
    xyz = rospy.Publisher('pca_linearMovement', Vector3, queue_size=10)
    rate = rospy.Rate(10) # 10hz
    
    # Variable type of float64 for x,y,z
    v = Vector3()
    v.y = data.axes[1]
    v.x = data.axes[0]
    xyz.publish(v)
    
    
def joystick_pub():
    # Initilizes a default ros node 
    rospy.init_node('joystick', anonymous=True)
    sub = rospy.Subscriber("joy", Joy, callback) # data recieved calls function callback()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    joystick_pub()