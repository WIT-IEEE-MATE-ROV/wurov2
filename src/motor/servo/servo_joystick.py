#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3
import time
from board import SCL, SDA
import busio
from adafruit_motor import servo
from adafruit_pca9685 import PCA9685

i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 50

servo0 = servo.Servo(pca.channels[0],min_pulse=1100, max_pulse=1900)
servo1 = servo.Servo(pca.channels[1],min_pulse=1100, max_pulse=1900)

# # We sleep in the loops to give the servo time to move into position.
# for i in range(180):
#     servo0.angle = i
#     time.sleep(0.03)
# for i in range(180):
#     servo0.angle = 180 - i
#     time.sleep(0.03)

# # You can also specify the movement fractionally.
# fraction = 0.0
# while fraction < 1.0:
#     servo0.fraction = fraction
#     fraction += 0.01
#     time.sleep(0.03)

# pca.deinit()

def map_range(x, in_min, in_max, out_min, out_max):
  return (x - in_min) * (out_max - out_min) // (in_max - in_min) + out_min

def callback(data):
    x = map_range(data.x,-1, 1, 0, 180)
    y = map_range(data.y,-1, 1, 0, 180)
    
    servo0.angle = x
    servo1.angle = y
    
    
    
def servo_joystick():
    # Initilizes a default ros node 
    rospy.init_node('servoControl', anonymous=True)
    sub = rospy.Subscriber("pca_linearMovement", Vector3, callback) # data recieved calls function callback()

    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    servo_joystick()