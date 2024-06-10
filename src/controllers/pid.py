import time

import numpy as np


class PIDController:
    # TODO: EWMA some inputs?
    # TODO: Derivative filter
    def __init__(self, p=0.0, i=0.0, d=0.0, f=0.0, i_zone=None, max_i_accum=None, max_output=None):
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
        # d_out = self.kD * (de / dt)
        d_out = 0
        f_out = self.kF * self.setpoint
        pid_output = p_out + i_out + d_out + f_out

        # Hard limit output
        if self.max_output is not None:
            pid_output = min(pid_output, self.max_output)

        return pid_output


class QuatPIDController:

    def __init__(self, p=0.0, i=0.0, d=0.0):
        self.kp = p
        self.ki = i
        self.kd = d
        self.setpoint = np.quaternion(1, 0, 0, 0)
        self.previous_time = time.time()
        self.prev_x_e = 0
        self.prev_y_e = 0
        self.prev_z_e = 0
        self.error = 0

    def set_setpoint(self, q):
        self.setpoint = q

    def get_quat_error(self, q=None, q_d=None):
        if q is not None and q_d is not None:
            self.error = q.inverse() * q_d
        return self.error

    def calculate(self, q):
        self.error = q.inverse() * self.setpoint
        # error =

        if self.error.w < 0:
            self.error = -self.error

        dt = time.time() - self.previous_time

        outputs = [0] * 3

        # outputs[0] = error.x * self.kp + ((error.x - self.prev_x_e) / dt) * self.kd
        # outputs[1] = error.y * self.kp + ((error.y - self.prev_y_e) / dt) * self.kd
        # outputs[2] = error.z * self.kp + ((error.z - self.prev_z_e) / dt) * self.kd

        if dt == 0:
            outputs[0] = -self.error.x * 1.1  # + ((error.x - self.prev_x_e) / dt) * 0.0
            outputs[1] = -self.error.y * 2.  # + ((error.y - self.prev_y_e) / dt) * 0.0
            outputs[2] = self.error.z * 3  # + ((error.z - self.prev_z_e) / dt) * 0.0
        else:
            outputs[0] = -(self.error.x * 1.1 + ((self.error.x - self.prev_x_e) / dt) * 0.0)
            outputs[1] = -(self.error.y * 2. + ((self.error.y - self.prev_y_e) / dt) * 0.0)
            outputs[2] = self.error.z * 3 + ((self.error.z - self.prev_z_e) / dt) * 1

        self.prev_x_e = self.error.x
        self.prev_y_e = self.error.y
        self.prev_z_e = self.error.z

        return outputs
