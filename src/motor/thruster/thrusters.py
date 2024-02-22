import numpy as np
from scipy.spatial.transform import Rotation
import math
from geometry_msgs.msg import Vector3, Twist, Quaternion
from pca import PCA9685
from queue import Queue
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
    """
    
    """
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


def get_horizontal_thruster_outputs(x, y, yaw):
    net_thrust_desired = np.array([x, y, yaw])
    return horizontal_factor @ net_thrust_desired


def get_thruster_outputs(x, y, z, roll, pitch, yaw, max_thrust=MAX_THRUST_KGF) -> np.array:
    horizontal = get_horizontal_thruster_outputs(x, y, yaw)
    vertical = get_vert_thruster_outputs(z, pitch, roll)
    outputs = np.concatenate((horizontal, vertical))
    desaturate_thrust_outputs(outputs, max_thrust)
    return outputs


# def get_thruster_outputs(net_thrust_xyz_ypr, max_thrust=MAX_THRUST_KGF):
#     return get_thruster_outputs(net_thrust_xyz_ypr[0], net_thrust_xyz_ypr[1], net_thrust_xyz_ypr[2],
#                                 net_thrust_xyz_ypr[3],net_thrust_xyz_ypr[4],net_thrust_xyz_ypr[5],
#                                 max_thrust=max_thrust)


def thrusts_to_us(thrusts: list):
    micros = [0] * len(thrusts)
    for i in range(len(thrusts)):
        reverse_thrust = thrusts[i] < 0
        if thrusts[i] > 0:
            micros[i] = quadratic_solve(thrusts[i], t100_right.c[0], t100_right.c[1], t100_right.c[2])[1 if reverse_thrust else 0]
        elif thrusts[i] < 0:
            micros[i] = quadratic_solve(-thrusts[i], t100_left.c[0], t100_left.c[1], t100_left.c[2])[1 if reverse_thrust else 0]
        else:
            micros[i] = 1500
    
    return micros


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

        
        def set_setpoint(self, q):
            self.setpoint = q


        def calculate(self, q):
            error = q.inverse() - self.setpoint

            if error.w < 0:
                error = -error

            dt = time.time() - self.previous_time
            
            outputs = [0] * 3
            outputs[0] = error.x * self.kp
            outputs[1] = error.y * self.kp
            outputs[2] = error.z * self.kp

            return outputs

        
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

        rot_euler = euler_from_quaternion(self.rotation_quat.x, self.rotation_quat.y, self.rotation_quat.z, self.rotation_quat.w)
        q = np.quaternion(self.rotation_quat.w, self.rotation_quat.x, self.rotation_quat.y, self.rotation_quat.z)
        self.quat_controller.set_setpoint(np.quaternion(1, 0, 0, 0))
        qc_output = self.quat_controller.calculate(q)
        d.angular.x = qc_output[0]
        d.angular.y = qc_output[1]
        d.angular.z = qc_output[2]


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

        print('Thruster outputs: FLH: %0.02f FRH: %0.02f BLH: %0.02f BRH: %0.02f FLV: %0.02f FRV: %0.02f BLV: %0.02f BRV: %0.02f' % 
        (thruster_outputs[0], thruster_outputs[1], thruster_outputs[2], thruster_outputs[3],
        thruster_outputs[4], thruster_outputs[5], thruster_outputs[6], thruster_outputs[7]))

        pca_outputs = thrusts_to_us(thruster_outputs)

            
        # print(f'PCA outputs: {pca_outputs}')
        # To automatically set stuff, make a dict of the ids then sort based off of slot number, then write adjacent stuff together
        self.__pca.set_us(Thrusters.__FLH_ID, pca_outputs)


if __name__ == '__main__':
    # yaw pitch roll
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
