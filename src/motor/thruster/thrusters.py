import numpy as np
import math
from pca import PCA9685

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


def get_vert_thruster_outputs_simple(upward_force, pitch, roll):
    return np.array([upward_force + pitch - roll,
                     upward_force + pitch + roll,
                     upward_force - pitch - roll,
                     upward_force - pitch + roll])


def get_horizontal_thruster_outputs(x, y, yaw):
    net_thrust_desired = np.array([x, y, yaw])
    return horizontal_factor @ net_thrust_desired


def get_thruster_outputs(x, y, z, yaw, pitch, roll, max_thrust=MAX_THRUST_KGF) -> np.array:
    horizontal = get_horizontal_thruster_outputs(x, y, yaw)
    vertical = get_vert_thruster_outputs(z, pitch, roll)
    outputs = np.concatenate((horizontal, vertical))
    desaturate_thrust_outputs(outputs, max_thrust)
    return outputs


def thrusts_to_us(thrusts: list):
    micros = [0] * len(thrusts)
    for i in range(len(thrusts)):
        reverse_thrust = thrusts[i] < 0
        if thrusts[i] > 0:
            pwm_us = quadratic_solve(thrusts[i], t100_right.c[0], t100_right.c[1], t100_right.c[2])[0 if reverse_thrust else 1]
        else:
            pwm_us = quadratic_solve(thrusts[i], t100_left.c[0], t100_left.c[1], t100_left.c[2])[0 if reverse_thrust else 1]
        micros[i] = pwm_us
    
    return micros


class Thrusters:
    # Horizontal thruster PCA slots
    __FLH_ID = 0
    __FRH_ID = 1
    __BLH_ID = 2
    __BRH_ID = 3
    # Vertical thruster PCA slots
    __FLV_ID = 4
    __FRV_ID = 5
    __BLV_ID = 6
    __BRV_ID = 7


    def __init__(self):
        self.__pca = PCA9685(0x40, 100, measured_frequency_hz=100)
        

    @property
    def pca(self):
        return self.__pca
    

    def set_thrust(self, x, y, z, yaw, pitch, roll):
        thruster_outputs = get_thruster_outputs(x, y, z, yaw, pitch, roll)
        pca_outputs = thrusts_to_us(thruster_outputs)
        # To automatically set stuff, make a dict of the ids then sort based off of slot number, then write adjacent stuff together
        self.__pca.set_us(Thrusters.__FLH_ID, pca_outputs)
        return thruster_outputs
