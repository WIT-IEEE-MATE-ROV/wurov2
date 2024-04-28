#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Twist, Quaternion
from thrusters import Thrusters
from light import Headlamp
from robo_arm import RoboArm
from pca import PCA9685
import time
import numpy as np
import math

LOOP_PERIOD_MS = 20.
desired_twist = Twist()
current_rotation = Quaternion()
light_on = False
arm_on = False
control_orientation = False
set_setpoint = False
scale = 4

def rotate_2d(x,y, angle_rad):
    x_p = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_p = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_p , y_p

def deadband(input, min_input):
    if abs(input) >= min_input:
        return (input - (abs(input) / input) * min_input) / (1 - min_input)
    else:
        return 0

def callback_joystick(data):
    global light_on, arm_on, control_orientation, set_setpoint
    # Does this once joystick sends data
    left_trigger = (-data.axes[2] + 1) / 2 
    right_trigger = (-data.axes[5] + 1) / 2 
    left_stick_x = data.axes[0]
    left_stick_y = data.axes[1]
    right_stick_x = data.axes[3]
    right_stick_y = data.axes[4]
    left_bumper = data.buttons[4]
    right_bumper = -data.buttons[5]
    a_button = data.buttons[0]
    start_button = data.buttons[0]

    left_stick_x, left_stick_y = rotate_2d(left_stick_x, left_stick_y, -np.pi/2)
    roll_bumper = left_bumper+right_bumper

    desired_twist.linear.x = deadband(left_stick_x, 0.1) * scale
    desired_twist.linear.y = deadband(left_stick_y, 0.1) * scale
    desired_twist.linear.z = deadband(right_trigger - left_trigger, 0.1) * scale # no press = 1, full press = -1
    desired_twist.angular.z = -deadband(right_stick_x, 0.1)  *2 # yaw
    desired_twist.angular.y = deadband(right_stick_y, 0.1) * scale# pitch
    desired_twist.angular.x = roll_bumper
    # print(f'right_trigger: {data.axes[5]}')
    if data.axes[7] == 1.0:
        light_on = True
    elif data.axes[7] == -1.0:
        light_on = False

    if data.axes[6] == 1.0:
        arm_on = True
    elif data.axes[6] == -1.0:
        arm_on = False

    if a_button:
        control_orientation = True
    else:
        control_orientation = False

    if start_button:
        set_setpoint = True
    else:
        set_setpoint = False


def callback_quat(data, thrusters):
    # print(f'thruster_node quat recieved: {data}')
    thrusters.set_rotation(data)

    
def thruster_pub():
    global desired_twist
    thrusters = Thrusters()
    pca_back = PCA9685(0x41, 100)
    pca_back.setup()

    headlamp = Headlamp(pca_back, 8)
    robo_arm = RoboArm(pca_back, 10)

    print(f'Thrusters started')

    rospy.init_node('thrusters', anonymous=True)
    joy_sub = rospy.Subscriber("joy", Joy, callback_joystick) # Gets data from joy msg in float32 array for axes and int32 array for buttons
    quat_sub = rospy.Subscriber("bno/quat", Quaternion, callback_quat, callback_args=(thrusters))

    

    rate = rospy.Rate(1 / (LOOP_PERIOD_MS / 1000))

    while not rospy.is_shutdown():
        thrusters.set_thrust(desired_twist, depth_lock=False)

        thrusters.update(control_orientation=control_orientation)

        headlamp.set_brightness(1.0 if light_on else 0.0)

        if arm_on:
            robo_arm.open()
        else:
            robo_arm.close()
        
        if set_setpoint:
            thrusters.set_test_rot_setpoint()
        
        # rate.sleep()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()
    desired_twist.linear.x = 0
    desired_twist.linear.y = 0
    desired_twist.linear.z = 0
    desired_twist.angular.z = 0
    desired_twist.angular.y = 0
    desired_twist.angular.x = 0
    thrusters.update()

if __name__ == '__main__':
    thruster_pub()
    
