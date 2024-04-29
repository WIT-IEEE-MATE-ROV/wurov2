import numpy as np
from scipy.spatial.transform import Rotation
import math
from geometry_msgs.msg import Vector3, Twist, Quaternion
from pca import PCA9685
from queue import Queue
from abc import ABC, abstractmethod
# from typing import override

from thruster_data import *

import sys
sys.path.append('/home/jetson/catkin_ws/src/wurov2_0/src/controllers')

from pid import PIDController, QuatPIDController

import quaternion
import time

# 1: Front Left
# 2: Front Right
# 3: Back Left
# 4: Back Right

THRUSTER_ANGLE_DEG = 45
thruster_angles = np.array(
    [THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG]) * np.pi / 180
THRUSTER_LENGTH_DISTANCE_M = 0.5842
THRUSTER_WIDTH_DISTANCE_M = 0.4826
THRUSTER_DIAGONAL_DISTANCE_M = math.sqrt(THRUSTER_WIDTH_DISTANCE_M ** 2 + THRUSTER_LENGTH_DISTANCE_M ** 2)
HALF_LENGTH = THRUSTER_LENGTH_DISTANCE_M / 2
HALF_WIDTH = THRUSTER_WIDTH_DISTANCE_M / 2
HALF_DIAGONAL = THRUSTER_DIAGONAL_DISTANCE_M / 2
LENGTH_DIAGONAL_ANGLE_RAD = -math.acos(THRUSTER_LENGTH_DISTANCE_M / THRUSTER_DIAGONAL_DISTANCE_M)

YAW_TANGENTIAL_FORCE = math.sin(
    thruster_angles[0] - LENGTH_DIAGONAL_ANGLE_RAD)

horizontal_thruster_config = np.array([[math.cos(thruster_angles[0]),
                                        math.cos(thruster_angles[1]),
                                        -math.cos(thruster_angles[2]),
                                        -math.cos(thruster_angles[3])],
                                       [math.sin(thruster_angles[0]),
                                        -math.sin(thruster_angles[1]),
                                        math.sin(thruster_angles[2]),
                                        -math.sin(thruster_angles[3])],
                                       [HALF_DIAGONAL * math.sin(
                                           thruster_angles[0] - LENGTH_DIAGONAL_ANGLE_RAD),
                                        -HALF_DIAGONAL * math.sin(
                                            thruster_angles[1] - LENGTH_DIAGONAL_ANGLE_RAD),
                                        -HALF_DIAGONAL * math.sin(
                                            thruster_angles[2] - LENGTH_DIAGONAL_ANGLE_RAD),
                                        HALF_DIAGONAL * math.sin(
                                            thruster_angles[3] - LENGTH_DIAGONAL_ANGLE_RAD)]])

h_U, h_S, h_V_T = np.linalg.svd(horizontal_thruster_config)
h_S = np.diag(h_S)
h_S_inv = np.linalg.inv(h_S)

h_V = np.transpose(h_V_T)
h_S_inv_0 = np.vstack([h_S_inv, [0, 0, 0]])
h_U_T = np.transpose(h_U)

# Assuming positive thrust forces up
vertical_thruster_config = np.array([[1, 1, 1, 1],
                                     [-HALF_LENGTH, -HALF_LENGTH, HALF_LENGTH, HALF_LENGTH],
                                     [-HALF_WIDTH, HALF_WIDTH, -HALF_WIDTH, HALF_WIDTH]])

v_U, v_S, v_V_T = np.linalg.svd(vertical_thruster_config)
v_S = np.diag(v_S)
v_S_inv = np.linalg.inv(v_S)

v_V = np.transpose(v_V_T)
v_S_inv_0 = np.vstack([v_S_inv, [0, 0, 0]])
v_U_T = np.transpose(v_U)

horizontal_factor = h_V @ h_S_inv_0 @ h_U_T
vertical_factor = v_V @ v_S_inv_0 @ v_U_T

# Constants to use for feedforward control
MAX_THRUST_KGF = 1.768181818
MAX_NET_X_KGF = MAX_THRUST_KGF * 4 * math.cos(thruster_angles[0])
MAX_NET_Y_KGF = MAX_THRUST_KGF * 4 * math.sin(thruster_angles[0])
MAX_NET_Z_KGF = MAX_THRUST_KGF * 4
MAX_NET_YAW_MOMENT_KGF = MAX_THRUST_KGF * 4 * YAW_TANGENTIAL_FORCE
MAX_NET_PITCH_MOMENT_KGF = MAX_THRUST_KGF * 4 * HALF_LENGTH
MAX_NET_ROLL_MOMENT_KGF = MAX_THRUST_KGF * 4 * HALF_WIDTH
MAX_DIAGONAL_THRUST = MAX_THRUST_KGF * 2

MAX_YAW_RATE_RAD_S = 1
MAX_PITCH_RATE_RAD_S = 1
MAX_ROLL_RATE_RAD_S = 1

NET_CURRENT_LIMIT = 20

# Logistic boolean input: input(t) = 1 / (1 + e^(-8 * (t - .5)))


def quadratic_solve(y, a, b, c):
    x1 = -b / (2 * a)
    x2 = math.sqrt(b ** 2 - 4 * a * (c - y)) / (2 * a)
    return (x1 + x2), (x1 - x2)


def desaturate_thrust_outputs(outputs, max_thrust):
    real_max_thrust = 0
    for i in range(len(outputs)):
        real_max_thrust = max(real_max_thrust, abs(outputs[i]))

    if real_max_thrust > max_thrust:
        for i in range(len(outputs)):
            outputs[i] = outputs[i] / real_max_thrust * max_thrust


class Thruster(ABC):
    @abstractmethod
    def get_us_from_thrust(self, thrust_kgf):
        pass


    @abstractmethod
    def get_current_from_us(self, us):
        pass


    @abstractmethod
    def get_us_from_current(self, current, reverse):
        pass


def current_limit(thrusters: "list[Thruster]", pwm_us: "list[float]", net_current_limit):
    current_sum = 0
    currents = [0] * len(thrusters)
    for i in range(len(thrusters)):
        c = thrusters[i].get_current_from_us(pwm_us[i])
        current_sum += c
        currents[i] = c

    if current_sum > net_current_limit:
        current_scale_back = net_current_limit / current_sum
        for i in range(len(thrusters)):
            pwm_us[i] = thrusters[i].get_us_from_current(currents[i] * current_scale_back, pwm_us[i] < 1500)
            

def get_vert_thruster_outputs(z_force, pitch, roll):
    forces = np.array([z_force, pitch, roll])
    return vertical_factor @ forces


def get_horizontal_thruster_outputs(x, y, yaw):
    net_thrust_desired = np.array([x, y, yaw])
    return horizontal_factor @ net_thrust_desired


def get_thruster_outputs(x, y, z, roll, pitch, yaw, max_thrust=MAX_THRUST_KGF) -> np.array:
    horizontal = get_horizontal_thruster_outputs(x, y, yaw)
    vertical = get_vert_thruster_outputs(z, pitch, roll)
    outputs = np.concatenate((horizontal, vertical))
    desaturate_thrust_outputs(outputs, max_thrust)
    return outputs


def rotate_2d(x, y, angle_rad):
    x_p = x * np.cos(angle_rad) - y * np.sin(angle_rad)
    y_p = x * np.sin(angle_rad) + y * np.cos(angle_rad)
    return x_p, y_p


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

    return roll_x, pitch_y, yaw_z  # in radians


def to_np_quat(q: Quaternion):
    return np.quaternion(q.w, q.x, q.y, q.z)


class T100Thruster(Thruster):
    t100_mid = int(len(t100_thrust_12v) / 2)
    t100_thrust_left = np.poly1d(np.polyfit(t100_pwm_value[:(t100_mid - 2)], t100_thrust_12v[:(t100_mid - 2)], 2))
    t100_thrust_right = np.poly1d(np.polyfit(t100_pwm_value[(t100_mid + 2):], t100_thrust_12v[(t100_mid + 2):], 2))
    t100_current_full = np.poly1d(np.polyfit(t100_pwm_value, t100_current_12v, 4))
    t100_current_left = np.poly1d(np.polyfit(t100_pwm_value[:(t100_mid - 2)], t100_current_12v[:(t100_mid - 2)], 2))
    t100_current_right = np.poly1d(np.polyfit(t100_pwm_value[(t100_mid + 2):], t100_current_12v[(t100_mid + 2):], 2))

    def get_us_from_thrust(self, thrust_kgf):
        reverse_thrust = thrust_kgf < 0

        if thrust_kgf > 0:
            micros = \
            quadratic_solve(thrust_kgf, T100Thruster.t100_thrust_right.c[0], T100Thruster.t100_thrust_right.c[1],
                            T100Thruster.t100_thrust_right.c[2])[1 if reverse_thrust else 0]
        elif thrust_kgf < 0:
            micros = \
            quadratic_solve(-thrust_kgf, T100Thruster.t100_thrust_left.c[0], T100Thruster.t100_thrust_left.c[1],
                            T100Thruster.t100_thrust_left.c[2])[1 if reverse_thrust else 0]
        else:
            micros = 1500

        return micros


    def get_current_from_us(self, us):
        if us < 1500 - 20 or us > 1500 + 20:
            return T100Thruster.t100_current_full(us)
        else:
            return 0.0
        

    def get_us_from_current(self, current, reverse):
        coeffs = T100Thruster.t100_current_full.c.copy()
        coeffs[-1] -= current
        solved = np.roots(coeffs)
        actual_roots = []
        for r in solved:
            if 1000 < r < 2000:
                actual_roots.append(r)

        if reverse:
            return min(actual_roots)
        else:
            return max(actual_roots)


class T200Thruster(Thruster):

    # abs the list of thrusts because the t100 thrust values are all positive
    for i in range(len(t200_thrust_12v)):
        t200_thrust_12v[i] = abs(t200_thrust_12v[i])

    t200_mid = int(len(t200_thrust_12v) / 2)
    t200_thrust_left = np.poly1d(np.polyfit(t200_pwm_value[:(t200_mid - 9)], t200_thrust_12v[:(t200_mid - 9)], 2))
    t200_thrust_right = np.poly1d(np.polyfit(t200_pwm_value[(t200_mid + 9):], t200_thrust_12v[(t200_mid + 9):], 2))
    t200_current_full = np.poly1d(np.polyfit(t200_pwm_value, t200_current_12v, 8))

    t200_current_left = np.poly1d(np.polyfit(t200_pwm_value[:(t200_mid - 9)], t200_current_12v[:(t200_mid - 9)], 2))
    t200_current_right = np.poly1d(np.polyfit(t200_pwm_value[(t200_mid + 9):], t200_current_12v[(t200_mid + 9):], 2))

    def get_us_from_thrust(self, thrust_kgf):
        reverse_thrust = thrust_kgf < 0

        if thrust_kgf > 0:
            micros = \
            quadratic_solve(thrust_kgf, T200Thruster.t200_thrust_right.c[0], T200Thruster.t200_thrust_right.c[1],
                            T200Thruster.t200_thrust_right.c[2])[1 if reverse_thrust else 0]
        elif thrust_kgf < 0:
            micros = \
            quadratic_solve(-thrust_kgf, T200Thruster.t200_thrust_left.c[0], T200Thruster.t200_thrust_left.c[1],
                            T200Thruster.t200_thrust_left.c[2])[1 if reverse_thrust else 0]
        else:
            micros = 1500

        return micros

    def get_current_from_us(self, us):
        if us < 1500 - 4 * 9 or us > 1500 + 4 * 9:
            return T200Thruster.t200_current_full(us)
        else:
            return 0.0


    def get_us_from_current(self, current, reverse):
        coeffs = T200Thruster.t200_current_full.c.copy()
        coeffs[-1] -= current
        solved = np.roots(coeffs)
        actual_roots = []
        for r in solved:
            if 1000 < r < 2000:
                actual_roots.append(r)

        if reverse:
            return min(actual_roots)
        else:
            return max(actual_roots)


class Thrusters:
    # Horizontal thruster PCA slots
    __FLH_ID = 0
    __FRH_ID = 5
    __BLH_ID = 1
    __BRH_ID = 6
    # Vertical thruster PCA slots__FLH_ID
    __FLV_ID = 3
    __FRV_ID = 4
    __BLV_ID = 2
    __BRV_ID = 7

    # Thruster creation
    flh_thruster = T100Thruster()
    frh_thruster = T100Thruster()
    blh_thruster = T100Thruster()
    brh_thruster = T100Thruster()
    flv_thruster = T100Thruster()
    frv_thruster = T100Thruster()
    blv_thruster = T100Thruster()
    brv_thruster = T100Thruster()
    all_thrusters = [flh_thruster, frh_thruster, blh_thruster, brh_thruster,
                     flv_thruster, frv_thruster, blv_thruster, brv_thruster]


    def __init__(self):
        self.__pca = PCA9685(0x40, 100, measured_frequency_hz=105.6)
        self.__pca.software_reset()
        self.__pca.setup()
        self.__pca.set_sleep(False)

        self.desired_twist = Twist()

        self.rotation_quat = Quaternion()
        self.desired_rotation = Quaternion(0, 0, 0, 1)
        self.rotation_offset = Quaternion()
        self.test_rot_setpoint = Quaternion()

        self.thust_outputs = [0] * 8
        self.us_outputs = [0] * 8

        self.previous_roll = 0
        self.previous_pitch = 0
        self.previous_yaw = 0

        # self.roll_controller = PIDController(p=0.1111, i=0, d=0)
        # self.pitch_controller = PIDController(p=0.1111)
        # self.yaw_controller = PIDController(p=0.11111)
        self.quat_controller = QuatPIDController(p=1.1)
        self.depth_controller = PIDController(p=0, d=0)


    @property
    def pca(self):
        return self.__pca


    def set_rotation(self, r: Quaternion) -> None:
        """
        Set the current ROV rotation

        Args:
            r (Quaternion): ROV rotation
        """
        self.rotation_quat = r

    
    def get_ros_rotation(self):
        return self.rotation_quat


    def set_desired_rotation(self, r_d: Quaternion) -> None:
        self.desired_rotation = r_d

    
    def set_rotation_offset(self, r_o: Quaternion):
        self.rotation_offset = r_o


    def get_current_rotation(self) -> np.quaternion:
        to_np_quat(self.rotation_offset).inverse() - to_np_quat(self.rotation_quat)

    
    def set_test_rot_setpoint(self):
        self.test_rot_setpoint = self.get_current_rotation()


    def get_roll_pitch_feedforwards(self, rot: Quaternion):
        r = euler_from_quaternion(rot)
        pitch_ff = math.sin(r.y) * HALF_LENGTH
        roll_ff = math.sin(r.x) * HALF_WIDTH
        return (roll_ff, pitch_ff)


    def get_thrust_outputs(self):
        return self.thrust_outputs
    

    def get_pwm_period_outputs(self):
        return self.us_outputs


    def set_thrust(self, thrust_twist: Twist, depth_lock: bool = False,
                   depth_command: float = None) -> None:
        """
        Set the desired thrust vector

        Args:
            x (float): ROV-relative x thrust
            y (float): ROV-relative y thrust
            z (float): ROV-relative z thrust
            roll (float): ROV-relative roll moment
            pitch (float): ROV-relative pitch moment
            yaw (float): ROV-relative yaw moment
            depth_lock (bool, optional): Keep the ROV at a constant depth. Defaults to False.
            depth_command (float, optional): Control the ROV depth thrust. Defaults to None.
        """

        if depth_lock:
            # q = [self.rotation_quat.x, self.rotation_quat.y, self.rotation_quat.z, self.rotation_quat.w]
            q = self.get_current_rotation()

            try:
                # Turn current rotation quat into scipy rotation
                q_arr = q.as_float_array() # w x y z
                current_rotation = Rotation.from_quat([q_arr[1], q_arr[2], q_arr[3], q_arr[0]])
            except ValueError:
                print('depth_lock not applied (invalid quat)')
                self.desired_twist = thrust_twist
                return

            # Keep only x and y components of the direction
            desired_direction = np.array([x, y, 0])

            # Rotate current rotation to the desired direction
            rov_direction = current_rotation.apply(desired_direction)
            x = rov_direction[0]
            y = rov_direction[1]
            z = rov_direction[2]

        self.desired_twist = thrust_twist


    def update(self, control_orientation=False) -> None:
        """
        Calculate PID outputs and set PCA PWM values
        """

        d = self.desired_twist

        if control_orientation:
            # q_r = np.quaternion(self.rotation_quat.w, self.rotation_quat.x, self.rotation_quat.y, self.rotation_quat.z)
            q_r = self.get_current_rotation()
            q_d = Thrusters.to_np_quat(self.test_rot_setpoint)
            self.quat_controller.set_setpoint(q_d)
            qc_output = self.quat_controller.calculate(q_r)
            d.angular.x = qc_output[0]
            d.angular.y = qc_output[1]
            d.angular.z = qc_output[2]


        thrust_outputs = get_thruster_outputs(d.linear.x, d.linear.y, d.linear.z, d.angular.x, d.angular.y, d.angular.z)

        print(
            'Thruster outputs: FLH: %0.04f FRH: %0.04f BLH: %0.04f BRH: %0.04f FLV: %0.04f FRV: %0.04f BLV: %0.04f BRV: %0.04f' %
            (thrust_outputs[0], thrust_outputs[1], thrust_outputs[2], thrust_outputs[3],
             thrust_outputs[4], thrust_outputs[5], thrust_outputs[6], thrust_outputs[7]))

        # Calculate PCA microsecond +period for each thruster
        us_outputs = [Thrusters.all_thrusters[i].get_us_from_thrust(thrust_outputs[i]) for i in range(len(thrust_outputs))]

        self.thrust_outputs = thrust_outputs
        self.us_outputs = us_outputs

        # Current limit to avoid exploding the robot
        # current_limit(Thrusters.all_thrusters, us_outputs, NET_CURRENT_LIMIT)

        # Re-order us_outputs to allow just one I2C block write
        us_outputs_reordered = [us_outputs[0], us_outputs[2], us_outputs[6], us_outputs[4], us_outputs[5],
                                us_outputs[1], us_outputs[3], us_outputs[7]]
        
        print(f'PCA outputs: {us_outputs_reordered}')

        self.__pca.set_us(Thrusters.__FLH_ID, us_outputs_reordered)


if __name__ == '__main__':
    # yaw pitch roll
    t = [0, 0, 0]

    r = [0,
         45 * math.pi / 180.,
         45 * math.pi / 180.]
    current_rotation = Rotation.from_euler('zyx', r)
    print('ROV rotation: (%.03f, %.03f, %.03f)' % (r[0], r[1], r[2]))

    # x y z velocities
    des = [0, 1, 0]
    desired_global_direction = np.array([des[0], des[1], des[2]])
    print('Desired Global: (%.03f, %.03f, %.03f)' % (
    desired_global_direction[0], desired_global_direction[1], desired_global_direction[2]))

    rov_relative_direction = current_rotation.apply(desired_global_direction)
    print('ROV net thrust:(%.03f, %.03f, %.03f)' % (
    rov_relative_direction[0], rov_relative_direction[1], rov_relative_direction[2]))
