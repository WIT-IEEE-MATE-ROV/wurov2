#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3, Twist
from pca import PCA9685
import time
import numpy as np
import math


t100_pwm_value = [
    1100, 1110, 1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190, 1200, 1210, 1220, 1230, 1240, 1250, 1260, 1270, 1280,
    1290, 1300, 1310, 1320, 1330, 1340, 1350, 1360, 1370, 1380, 1390, 1400, 1410, 1420, 1430, 1440, 1450, 1460, 1470,
    1480, 1500, 1510, 1520, 1530, 1540, 1550, 1560, 1570, 1580, 1590, 1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670,
    1680, 1690, 1700, 1710, 1720, 1730, 1740, 1750, 1760, 1770, 1780, 1790, 1800, 1810, 1820, 1830, 1840, 1850, 1860,
    1870, 1880, 1890, 1900
]

t100_thrust_12v = [1.768181818, 1.640909091, 1.577272727, 1.527272727, 1.440909091, 1.4, 1.322727273, 1.259090909,
                   1.209090909, 1.163636364, 1.104545455, 1.040909091, 0.990909091, 0.927272727, 0.854545455,
                   0.790909091, 0.754545455, 0.704545455, 0.668181818, 0.622727273, 0.581818182, 0.531818182,
                   0.472727273, 0.427272727, 0.4, 0.368181818, 0.327272727, 0.272727273, 0.231818182, 0.2, 0.168181818,
                   0.140909091, 0.104545455, 0.072727273, 0.05, 0.031818182, 0.013636364, 0.009090909, 0, 0, 0, 0,
                   0.009090909, 0.036363636, 0.063636364, 0.104545455, 0.145454545, 0.195454545, 0.254545455,
                   0.309090909, 0.368181818, 0.431818182, 0.481818182, 0.545454545, 0.613636364, 0.686363636,
                   0.736363636, 0.804545455, 0.881818182, 0.963636364, 1.059090909, 1.131818182, 1.186363636,
                   1.254545455, 1.304545455, 1.386363636, 1.490909091, 1.577272727, 1.654545455, 1.727272727,
                   1.822727273, 1.959090909, 2.045454545, 2.1, 2.181818182, 2.263636364, 2.322727273, 2.418181818,
                   2.486363636, 2.518181818]

t100_mid = int(len(t100_thrust_12v) / 2)
t100_left = np.poly1d(np.polyfit(t100_pwm_value[:(t100_mid - 2)], t100_thrust_12v[:(t100_mid - 2)], 2))
t100_right = np.poly1d(np.polyfit(t100_pwm_value[(t100_mid + 2):], t100_thrust_12v[(t100_mid + 2):], 2))

def quadratic_solve(y, a, b, c):
    x1 = -b / (2 * a)
    x2 = math.sqrt(b ** 2 - 4 * a * (c - y)) / (2 * a)
    return (x1 + x2), (x1 - x2)

def pwm_to_thrust(pwm_on, voltage, use_t100=True):
    reversed_pwm = pwm_on < 1500

    if reversed_pwm:
        t_left = t100_left
        t_right = t100_right
    else:
        t_left = t100_right
        t_right = t100_left

    y = t_left(pwm_on)
    right_x = quadratic_solve(y, t_right.coeffs[0], t_right.coeffs[1], t_right.coeffs[2])[0 if reversed_pwm else 1]

    return (pwm_on, y), (right_x, y)
    

# --Horizontal Walsh matrix solve setup--
motor_angle = 45 * (math.pi / 180)
robot_length = 10
robot_width = 10
motor_origin_hypot = math.sqrt(robot_width ** 2 + robot_length ** 2) / 2
left_phi = math.asin((-robot_width / 2) / motor_origin_hypot)
right_phi = math.asin((robot_width / 2) / motor_origin_hypot)

q_inv = np.diag([1 / math.sin(motor_angle),
                1 / math.cos(motor_angle),
                1 / (motor_origin_hypot * math.sin(motor_angle - left_phi))])

W = np.array([[1, 1, 1, 1],
              [1, 1, -1, -1],
              [1, -1, 1, -1],
              [1, -1, -1, 1]])

# --Vertical Thruster SVD solve setup--
half_len = robot_length / 2
half_width = robot_width / 2
# Assuming positive thrust forces up
vert_T = np.array([[1, 1, 1, 1],
                   [-half_len, -half_len, half_len, half_len],
                   [-half_width, half_width, -half_width, half_width]])

U, S, V_T = np.linalg.svd(vert_T)
S = np.diag(S)
# print(U)
# print(S)
# print(V_T)
# exit(0)
S_inv = np.linalg.inv(S)

V = np.transpose(V_T)
S_inv_0 = np.vstack([S_inv, [0, 0, 0]])
U_T = np.transpose(U)

desired_twist = Twist()


def get_vert_thruster_outputs(z_force, pitch, roll):
    forces = np.array([z_force, pitch, roll])
    return V @ S_inv_0 @ U_T @ forces


def get_vert_thruster_outputs_simple(upward_force, pitch, roll):
    return np.array([upward_force + pitch - roll,
                     upward_force + pitch + roll,
                     upward_force - pitch - roll,
                     upward_force - pitch + roll])


def get_horizontal_thruster_outputs(x, y, theta):
    thrust_desired = np.array([x, y, theta])

    a = q_inv @ thrust_desired

    r = np.insert(a, 0, np.array([0]))

    outputs = .25 * W @ r

    return outputs

def rotate_2d(x,y, angle_rad):
    x_p = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_p = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_p , y_p

def callback_navigation(data):
    # Does this once joystick sends data
    left_trigger = (-data.axes[2] + 1) / 2 
    right_trigger = (-data.axes[5] + 1) / 2 
    left_stick_x = data.axes[0]
    left_stick_y = data.axes[1]

    left_stick_x, left_stick_y = rotate_2d(left_stick_x, left_stick_y, -np.pi/2)

    desired_twist.linear.x = left_stick_x
    desired_twist.linear.y = left_stick_y
    desired_twist.linear.z = right_trigger - left_trigger # no press = 1, full press = -1
    # print(f'right_trigger: {data.axes[5]}')
    
    
def callback_angle(data):
    # Does this once Imu data comes in
    # desired_twist.angular.x = data.x # Roll degress
    # desired_twist.angular.y = data.y # pitch degress
    # desired_twist.angular.z = data.z # yaw degress
    pass
    
def thruster_pub():
    global desired_twist
    # Initilizes a default ros node 
    rospy.init_node('thrusters', anonymous=True)
    sub = rospy.Subscriber("joy", Joy, callback_navigation) # Gets data from joy msg in float32 array for axes and int32 array for buttons
    sub2 = rospy.Subscriber("orientation", Vector3, callback_angle) # Has data.x as roll = x, pitch = y, yaw = z in degrees
    rate = rospy.Rate(10) # 10hz
    
    # pca = PCA9685(0x40, 100)

    while not rospy.is_shutdown():
        horizontal_output = get_horizontal_thruster_outputs(desired_twist.linear.x, desired_twist.linear.y, desired_twist.angular.z)
        vertical_output = get_vert_thruster_outputs(desired_twist.linear.z, desired_twist.angular.y, desired_twist.angular.x)

        print(f'horizontal_output: {horizontal_output}\nvertical_output: {vertical_output}\n\n')

        # # Example workflow
        # # Error and kP values
        # error = 0
        # pitch_error = 0
        # pitch_kP = 0
        # kp = 0

        # pitch_output = pitch_kP * pitch_error

        # vertical_output = get_vert_thruster_outputs(0, pitch_output, 0)
        # pca.set_duty_cycle(0, vertical_output[0])
        # pca.set_duty_cycle(1, vertical_output[1])


        # # Calculate PID output
        # p_out = error * kp
        
        # # Clamp output from -1 to 1
        # p_out = max(min(p_out, 1), -1)

        # # Send output to PCA at slot 0
        # pca.set_duty_cycle(0, p_out)
        
        rate.sleep()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    thruster_pub()