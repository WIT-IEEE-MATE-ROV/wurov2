#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64
from sensor_msgs.msg import Joy
from geometry_msgs.msg import Vector3
import numpy as np
import math

# --Horizontal Walsh matrix solve setup--
motor_angle = 45 * (math.pi / 180)
robot_length = 10
robot_width = 10
motor_origin_hypot = math.sqrt(robot_width ** 2 + robot_length ** 2) / 2
left_phi = math.asin((-robot_width / 2) / motor_origin_hypot)
right_phi = math.asin((robot_width / 2) / motor_origin_hypot)

q_inv = np.diag([1 / math.sin(motor_angle), 1 / math.cos(motor_angle),
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


def callback(data):
    # xyz = rospy.Publisher('pca_linearMovement', Vector3, queue_size=10)
    # rate = rospy.Rate(10) # 10hz
    
    # Array element of data.axes 0 and 1 are Joystick button 3
    v = Vector3()
    v.y = data.axes[1]
    v.x = data.axes[0]
    
    
    
def thruster_pub():
    # Initilizes a default ros node 
    rospy.init_node('thrusters', anonymous=True)
    sub = rospy.Subscriber("joy", Joy, callback) # data recieved calls function callback()
    # movement
    print("Hello")

    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    thruster_pub()