import numpy as np
from scipy.spatial.transform import Rotation
import math
from geometry_msgs.msg import Vector3, Twist, Quaternion
from pca import PCA9685
from queue import Queue
from abc import ABC, abstractmethod
from typing import override
import quaternion
import time

# 1: Front Left
# 2: Front Right
# 3: Back Left
# 4: Back Right

# TODO: Redefine these parameters once robot is ready
THRUSTER_ANGLE_DEG = 45
# THRUSTER_ANGLE_DEG = math.acos(.875) * 180/np.pi # Sanity check angle
thruster_angles = np.array(
    [THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG, THRUSTER_ANGLE_DEG]) * np.pi / 180
THRUSTER_LENGTH_DISTANCE_M = 1
THRUSTER_WIDTH_DISTANCE_M = .5
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
MAX_THRUST_KGF = 1.5
MAX_NET_X_KGF = MAX_THRUST_KGF * 4 * math.cos(thruster_angles[0])
MAX_NET_Y_KGF = MAX_THRUST_KGF * 4 * math.sin(thruster_angles[0])
MAX_NET_Z_KGF = MAX_THRUST_KGF * 4
MAX_NET_YAW_MOMENT_KGF = MAX_THRUST_KGF * 4 * YAW_TANGENTIAL_FORCE
MAX_NET_PITCH_MOMENT_KGF = MAX_THRUST_KGF * 4 * HALF_LENGTH
MAX_NET_ROLL_MOMENT_KGF = MAX_THRUST_KGF * 4 * HALF_WIDTH
# Logistic boolean input: input(t) = 1 / (1 + e^(-8 * (t - .5)))


def quadratic_solve(y, a, b, c):
    """
    
    """
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
    return x_p , y_p


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


class Thruster(ABC):
    @abstractmethod
    def get_us(thrust_kgf):
        pass


class T100Thruster(Thruster):
    t100_pwm_value = [1100, 1110, 1120, 1130, 1140, 1150, 1160, 1170, 1180, 1190, 1200, 1210, 1220, 1230, 1240, 1250, 1260, 1270, 1280,
                      1290, 1300, 1310, 1320, 1330, 1340, 1350, 1360, 1370, 1380, 1390, 1400, 1410, 1420, 1430, 1440, 1450, 1460, 1470,
                      1480, 1500, 1510, 1520, 1530, 1540, 1550, 1560, 1570, 1580, 1590, 1600, 1610, 1620, 1630, 1640, 1650, 1660, 1670,
                      1680, 1690, 1700, 1710, 1720, 1730, 1740, 1750, 1760, 1770, 1780, 1790, 1800, 1810, 1820, 1830, 1840, 1850, 1860,
                      1870, 1880, 1890, 1900]

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
    t100_left = np.poly1d(np.polyfit(t100_pwm_value[:(t100_mid - 2)],  t100_thrust_12v[:(t100_mid - 2)], 2))
    t100_right = np.poly1d(np.polyfit(t100_pwm_value[(t100_mid + 2):], t100_thrust_12v[(t100_mid + 2):], 2))

    @override
    def get_us(self, thrust_kgf):
        reverse_thrust = thrust_kgf < 0

        if thrust_kgf > 0:
            micros = quadratic_solve(thrust_kgf, T100Thruster.t100_right.c[0], T100Thruster.t100_right.c[1], T100Thruster.t100_right.c[2])[1 if reverse_thrust else 0]
        elif thrust_kgf < 0:
            micros = quadratic_solve(-thrust_kgf, T100Thruster.t100_left.c[0], T100Thruster.t100_left.c[1], T100Thruster.t100_left.c[2])[1 if reverse_thrust else 0]
        else:
            micros = 1500
        
        return micros
    

class T200Thruster(Thruster):
    t200_pwm_value = [1100, 1104, 1108, 1112, 1116, 1120, 1124, 1128, 1132, 1136, 1140, 1144, 1148, 1152, 1156, 1160, 1164,
                      1168, 1172, 1176, 1180, 1184, 1188, 1192, 1196, 1200, 1204, 1208, 1212, 1216, 1220, 1224, 1228, 1232,
                      1236, 1240, 1244, 1248, 1252, 1256, 1260, 1264, 1268, 1272, 1276, 1280, 1284, 1288, 1292, 1296, 1300,
                      1304, 1308, 1312, 1316, 1320, 1324, 1328, 1332, 1336, 1340, 1344, 1348, 1352, 1356, 1360, 1364, 1368,
                      1372, 1376, 1380, 1384, 1388, 1392, 1396, 1400, 1404, 1408, 1412, 1416, 1420, 1424, 1428, 1432, 1436,
                      1440, 1444, 1448, 1452, 1456, 1460, 1464, 1468, 1472, 1476, 1480, 1484, 1488, 1492, 1496, 1500, 1504,
                      1508, 1512, 1516, 1520, 1524, 1528, 1532, 1536, 1540, 1544, 1548, 1552, 1556, 1560, 1564, 1568, 1572,
                      1576, 1580, 1584, 1588, 1592, 1596, 1600, 1604, 1608, 1612, 1616, 1620, 1624, 1628, 1632, 1636, 1640,
                      1644, 1648, 1652, 1656, 1660, 1664, 1668, 1672, 1676, 1680, 1684, 1688, 1692, 1696, 1700, 1704, 1708,
                      1712, 1716, 1720, 1724, 1728, 1732, 1736, 1740, 1744, 1748, 1752, 1756, 1760, 1764, 1768, 1772, 1776,
                      1780, 1784, 1788, 1792, 1796, 1800, 1804, 1808, 1812, 1816, 1820, 1824, 1828, 1832, 1836, 1840, 1844,
                      1848, 1852, 1856, 1860, 1864, 1868, 1872, 1876, 1880, 1884, 1888, 1892, 1896, 1900]

    t200_thrust_12v = [-2.90, -2.92, -2.89, -2.83, -2.79, -2.76, -2.72, -2.67, -2.60, -2.59, -2.56, -2.49, -2.44, -2.43,
                       -2.39, -2.34, -2.30, -2.25, -2.23, -2.18, -2.14, -2.10, -2.07, -2.01, -1.98, -1.95, -1.88, -1.85,
                       -1.81, -1.78, -1.73, -1.66, -1.65, -1.61, -1.56, -1.53, -1.49, -1.47, -1.44, -1.40, -1.37, -1.33,
                       -1.29, -1.28, -1.22, -1.19, -1.15, -1.12, -1.08, -1.04, -1.02, -0.99, -0.96, -0.93, -0.90, -0.87,
                       -0.83, -0.79, -0.77, -0.74, -0.72, -0.69, -0.66, -0.64, -0.60, -0.57, -0.54, -0.52, -0.49, -0.47,
                       -0.44, -0.42, -0.39, -0.37, -0.34, -0.32, -0.29, -0.27, -0.24, -0.23, -0.20, -0.18, -0.16, -0.15,
                       -0.12, -0.11, -0.09, -0.07, -0.06, -0.05, -0.04, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00,
                       0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.00, 0.04, 0.05, 0.07, 0.10, 0.11, 0.13,
                       0.15, 0.18, 0.20, 0.22, 0.25, 0.28, 0.31, 0.33, 0.37, 0.39, 0.43, 0.46, 0.49, 0.52, 0.55, 0.59, 0.63,
                       0.65, 0.68, 0.71, 0.76, 0.79, 0.83, 0.86, 0.89, 0.93, 0.97, 1.00, 1.04, 1.08, 1.14, 1.16, 1.20, 1.23,
                       1.28, 1.31, 1.35, 1.40, 1.43, 1.48, 1.53, 1.56, 1.63, 1.67, 1.71, 1.77, 1.82, 1.85, 1.91, 1.92, 1.96,
                       2.03, 2.09, 2.13, 2.18, 2.24, 2.27, 2.33, 2.40, 2.46, 2.51, 2.56, 2.62, 2.65, 2.71, 2.78, 2.84, 2.87,
                       2.93, 3.01, 3.04, 3.08, 3.16, 3.23, 3.26, 3.32, 3.38, 3.40, 3.45, 3.50, 3.57, 3.64, 3.71, 3.69, 3.71]

    t200_mid = int(len(t200_thrust_12v) / 2)
    t200_left = np.poly1d(np.polyfit(t200_pwm_value[:(t200_mid - 9)],  t200_thrust_12v[:(t200_mid - 9)], 2))
    t200_right = np.poly1d(np.polyfit(t200_pwm_value[(t200_mid + 9):], t200_thrust_12v[(t200_mid + 9):], 2))


    @override
    def get_us(self, thrust_kgf):
        reverse_thrust = thrust_kgf < 0

        if thrust_kgf > 0:
            micros = quadratic_solve(thrust_kgf, T200Thruster.t200_right.c[0], T200Thruster.t200_right.c[1], T200Thruster.t200_right.c[2])[1 if reverse_thrust else 0]
        elif thrust_kgf < 0:
            micros = quadratic_solve(-thrust_kgf, T200Thruster.t200_left.c[0], T200Thruster.t200_left.c[1], T200Thruster.t200_left.c[2])[1 if reverse_thrust else 0]
        else:
            micros = 1500
        
        return micros


class PIDController:
    # TODO: EWMA some inputs?
    # TODO: Derivative filter
    def __init__(self, p=0, i=0, d=0, f=0, i_zone=None, max_i_accum=None, max_output=None):
        self.kP = p
        self.kI = i
        self.kD = d
        self.kF = f

        self.previous_error = 0
        self.previous_time = time.time()

        self.i_accum = 0
        self.setpoint = 0
        self.max_output = max_output

        self.i_zone = i_zone
        self.max_i_accum = max_i_accum
        self.max_output = max_output

    
    def set_setpoint(self, s):
        self.setpoint = s
        self.i_accum = 0


    def calculate(self, state):
        current_time = time.time()
        dt = current_time - self.previous_time
        self.previous_time = current_time

        error = self.setpoint - state
        de = error - self.previous_error

        # Clegg Integrator: Reset I term when error crosses 0
        if self.kF == 0 and np.sign(self.previous_error) != np.sign(error):
            self.i_accum = 0

        # Apply integration zone. 
        if self.i_zone is not None:
            if abs(error) < self.i_zone:
                self.i_accum += error * dt
            else:
                self.i_accum = 0
        else:
            self.i_accum += error * dt

        # Hard limit integral windup
        if self.max_i_accum is not None:
            self.i_accum = min(self.i_accum, self.max_i_accum)
        
        # Calculate PIDF terms
        p_out = self.kP * error
        i_out = self.kI * self.i_accum
        d_out = self.kD * (de / dt)
        f_out = self.kF * self.setpoint
        pid_output = p_out + i_out + d_out + f_out

        # Hard limit output
        if self.max_output is not None:
            pid_output = min(pid_output, self.max_output)
        
        return pid_output


class QuatPIDController:

        def __init__(self, p=0, i=0, d=0):
            self.kp = p
            self.ki = i
            self.kd = d
            self.setpoint = None
            self.previous_time = time.time()
            self.prev_x_e = 0
            self.prev_y_e = 0
            self.prev_z_e = 0

        
        def set_setpoint(self, q):
            self.setpoint = q


        def calculate(self, q):
            error = q.inverse() - self.setpoint

            if error.w < 0:
                error = -error

            dt = time.time() - self.previous_time
            
            outputs = [0] * 3
            outputs[0] = error.x * self.kp + (error.x - self.prev_x_e / dt) * self.kd
            outputs[1] = error.y * self.kp + (error.y - self.prev_y_e / dt) * self.kd
            outputs[2] = error.z * self.kp + (error.z - self.prev_z_e / dt) * self.kd
            self.prev_x_e = error.x
            self.prev_y_e = error.y
            self.prev_z_e = error.z
            
            return outputs

        
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

        self.previous_roll = 0
        self.previous_pitch = 0
        self.previous_yaw = 0

        self.roll_controller = PIDController(p=0.1111, i=0, d=0)
        self.pitch_controller = PIDController(p=0.1111)
        self.yaw_controller = PIDController(p=0.11111)
        self.quat_controller = QuatPIDController(p=0.7)
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
    

    def set_thrust(self, x:float, y:float, z:float, roll:float, pitch:float, yaw:float, depth_lock:bool=False, depth_command:float=None) -> None:
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
            q = [self.rotation_quat.x, self.rotation_quat.y, self.rotation_quat.z, self.rotation_quat.w]

            try:
                # Turn current rotation quat into scipy rotation
                current_rotation = Rotation.from_quat(q)
            except ValueError:
                print('depth_lock not applied (invalid quat)')
                self.desired_twist = Twist(Vector3(x, y, z), Vector3(roll, pitch, yaw))
                return

            # Keep only x and y components of the direction
            desired_direction = np.array([x, y, 0])

            # Rotate current rotation to the desired direction
            rov_direction = current_rotation.apply(desired_direction)
            x = rov_direction[0]
            y = rov_direction[1]
            z = rov_direction[2]

        self.desired_twist = Twist(Vector3(x, y, z), Vector3(roll, pitch, yaw))


    # def set_thrust(self, t: Twist, depth_lock=False):
    #     self.set_thrust(t.linear.x, t.linear.y, t.linear.z, t.angular.z, t.angular.y, t.angular.x, depth_lock=depth_lock)


    def update(self) -> None:
        """
        Calculate PID outputs and set PCA PWM values
        """

        d = self.desired_twist

        # rot_euler = euler_from_quaternion(self.rotation_quat.x, self.rotation_quat.y, self.rotation_quat.z, self.rotation_quat.w)
        # q = np.quaternion(self.rotation_quat.w, self.rotation_quat.x, self.rotation_quat.y, self.rotation_quat.z)
        # self.quat_controller.set_setpoint(np.quaternion(1, 0, 0, 0))
        # qc_output = self.quat_controller.calculate(q)
        # d.angular.x = qc_output[0]
        # d.angular.y = qc_output[1]
        # d.angular.z = qc_output[2]


        # if self.desired_twist.angular.x < 0.01:
        #     self.roll_controller.set_setpoint(self.previous_roll)
        #     d.angular.x = self.roll_controller.calculate(rot_euler[0])
        # else:
        #     self.previous_roll = rot_euler[0]
        
        # if self.desired_twist.angular.y < 0.01:
        #     self.pitch_controller.set_setpoint(self.previous_pitch)
        #     d.angular.y = self.pitch_controller.calculate(rot_euler[1])
        # else:
        #     self.previous_pitch = rot_euler[1]

        # if self.desired_twist.angular.z < 0.01:
        #     self.yaw_controller.set_setpoint(self.previous_yaw)
        #     d.angular.z = self.yaw_controller.calculate(rot_euler[2])
        # else:
        #     self.previous_yaw = rot_euler[2]

        thruster_outputs = get_thruster_outputs(d.linear.x, d.linear.y, d.linear.z, d.angular.x, d.angular.y, d.angular.z)

        print('Thruster outputs: FLH: %0.04f FRH: %0.04f BLH: %0.04f BRH: %0.04f FLV: %0.04f FRV: %0.04f BLV: %0.04f BRV: %0.04f' % 
        (thruster_outputs[0], thruster_outputs[1], thruster_outputs[2], thruster_outputs[3],
        thruster_outputs[4], thruster_outputs[5], thruster_outputs[6], thruster_outputs[7]))

        
        pca_outputs = thrusts_to_us(thruster_outputs)
        pca_outputs = [t for t in thruster_outputs]

            
        print(f'PCA outputs: {pca_outputs}')
        # To automatically set stuff, reorder pca_outputs according to the pca slots
        # TODO: Reorder pca_outputs to allow for one call to set_us
        self.__pca.set_us(Thrusters.__FLH_ID, [pca_outputs[0]])
        self.__pca.set_us(Thrusters.__FRH_ID, [pca_outputs[1]])
        self.__pca.set_us(Thrusters.__BLH_ID, [pca_outputs[2]])
        self.__pca.set_us(Thrusters.__BRH_ID, [pca_outputs[3]])
        self.__pca.set_us(Thrusters.__FLV_ID, [pca_outputs[4]])
        self.__pca.set_us(Thrusters.__FRV_ID, [pca_outputs[5]])
        self.__pca.set_us(Thrusters.__BLV_ID, [pca_outputs[6]])
        self.__pca.set_us(Thrusters.__BRV_ID, [pca_outputs[7]])


if __name__ == '__main__':
    # yaw pitch roll
    t = [0, 0, 0]
    t_r = thrusts_to_us(t)
    print(t_r)

    r = [0, 
         45 * math.pi / 180., 
         45 * math.pi / 180.]
    current_rotation = Rotation.from_euler('zyx', r)
    print('ROV rotation: (%.03f, %.03f, %.03f)' % (r[0], r[1], r[2]))

    # x y z velocities
    des = [0, 1, 0]
    desired_global_direction = np.array([des[0], des[1], des[2]])
    print('Desired Global: (%.03f, %.03f, %.03f)' % (desired_global_direction[0], desired_global_direction[1], desired_global_direction[2]))

    rov_relative_direction = current_rotation.apply(desired_global_direction)
    print('ROV net thrust:(%.03f, %.03f, %.03f)' % (rov_relative_direction[0], rov_relative_direction[1], rov_relative_direction[2]))
