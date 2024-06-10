#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64, Float32MultiArray, MultiArrayLayout, MultiArrayDimension
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Twist, Quaternion
from thrusters import Thrusters, MAX_NET_X_KGF, MAX_ROLL_RATE_RAD_S
from light import Headlamp
from robo_arm import RoboArm
from pca import PCA9685
import time
import numpy as np
import math

LOOP_PERIOD_MS = 20.
desired_twist = Twist(Vector3(0, 0, 0), Vector3(0, 0, 0))
depth_command = 0
current_rotation = Quaternion()
light_on = False
arm_on = False
control_orientation = True
set_setpoint = True
set_offset = False
linear_scale = MAX_NET_X_KGF
rotation_scale = MAX_ROLL_RATE_RAD_S
depth_setpoint = 0
previous_value = 0
previous_time = time.time()

def rotate_2d(x,y, angle_rad):
    x_p = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_p = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_p , y_p

def deadband(input, min_input):
    if abs(input) >= min_input:
        return (input - (abs(input) / input) * min_input) / (1 - min_input)
    else:
        return 0
    
def clamp(input, min_val, max_val):
    return min(max(input, min_val), max_val)

def callback_joystick(data):
    global light_on, arm_on, control_orientation, set_setpoint, set_offset, previous_time, previous_value, depth_command
    # Does this once joystick sends data
    left_trigger = (data.axes[2] + 1) / 2 
    right_trigger = (data.axes[5] + 1) / 2 
    left_stick_x = -data.axes[0]
    left_stick_y = data.axes[1]
    right_stick_x = -data.axes[3]
    right_stick_y = data.axes[4]
    left_bumper = data.buttons[4]
    right_bumper = -data.buttons[5]
    a_button = data.buttons[0]
    b_button = data.buttons[1]
    x_button = data.buttons[2] 
    start_button = data.buttons[7]

    left_stick_x, left_stick_y = rotate_2d(left_stick_x, left_stick_y, -np.pi/2)
    roll_bumper = left_bumper + right_bumper

    # rate_limit = .1
    # current_time = time.time()
    # elapsed = current_time - previous_time
    # previous_value += clamp(roll_bumper - previous_value, -rate_limit * elapsed, rate_limit * elapsed)
    # previous_time = current_time

    desired_twist.linear.x = deadband(left_stick_x, 0.1) * linear_scale
    desired_twist.linear.y = deadband(left_stick_y, 0.1) * linear_scale
    # desired_twist.linear.z = deadband(right_trigger - left_trigger, 0.1) * linear_scale # no press = 1, full press = -1
    desired_twist.linear.z = 0 # no press = 1, full press = -1

    desired_twist.angular.z = -deadband(right_stick_x, 0.1)  * rotation_scale # yaw
    desired_twist.angular.y = deadband(right_stick_y, 0.1) * rotation_scale# pitch
    desired_twist.angular.x = roll_bumper * 0.5

    depth_command = deadband(right_trigger - left_trigger, 0.1) * linear_scale
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
        set_offset = True
    else:
        set_offset = False

    if b_button:
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

    dim_properties = MultiArrayDimension()
    dim_properties.label = 'kgf'
    dim_properties.size = 1
    dim_properties.stride = 8
    layout = MultiArrayLayout()
    layout.dim = [dim_properties]
    layout.data_offset = 0
    thrust_array = Float32MultiArray()
    thrust_array.layout = layout
    thrust_array.data = [0] * 8

    micros_array = Float32MultiArray()
    micros_array.layout = layout
    micros_array.data = [0] * 8

    thruster_rot = Vector3()
    thruster_quat = Quaternion()
    setpoint_rot = Vector3()

    error_rot = Vector3()

    thrust_pub = rospy.Publisher('thruster_node/thrust_kgf', Float32MultiArray, queue_size=3)
    micros_pub = rospy.Publisher('thruster_node/micros', Float32MultiArray, queue_size=3)
    thruster_rot_euler_pub = rospy.Publisher('thruster_node/rot_euler', Vector3, queue_size=3)
    thruster_rot_quat_pub = rospy.Publisher('thruster_node/rot_quat', Quaternion, queue_size=3)

    rot_setpoint_pub = rospy.Publisher('thruster_node/setpoint', Vector3, queue_size=3)

    error_pub = rospy.Publisher('thruster_node/rot_error', Vector3, queue_size=3)

    # TODO: Publish thruster node calculated depth.
    rate = rospy.Rate(1 / (LOOP_PERIOD_MS / 1000))

    while not rospy.is_shutdown():
        thrusters.set_thrust(desired_twist, xy_drive=True, depth_command=depth_command)

        thrusters.update(control_orientation=control_orientation)

        headlamp.set_brightness(1.0 if light_on else 0.0)

        if arm_on:
            robo_arm.open()
        else:
            robo_arm.close()
        
        if set_setpoint:
            thrusters.set_test_rot_setpoint()

        if set_offset:
            thrusters.set_rotation_offset()
        
        thrust_array.data = thrusters.get_thrust_outputs()
        micros_array.data = thrusters.get_pwm_period_outputs()
        thruster_rot = thrusters.get_ros_current_rotation_euler()
        thruster_quat = thrusters.get_ros_quat()
        setpoint_rot = thrusters.get_ros_test_rot_setpoint_euler()
        error_rot = thrusters.get_rot_error_euler()

        thrust_pub.publish(thrust_array)
        micros_pub.publish(micros_array)
        thruster_rot_euler_pub.publish(thruster_rot)
        thruster_rot_quat_pub.publish(thruster_quat)
        rot_setpoint_pub.publish(setpoint_rot)
        error_pub.publish(error_rot)
        #rate.sleep()
    
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
    
