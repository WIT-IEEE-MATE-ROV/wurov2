# SPDX-FileCopyrightText: 2020 Bryan Siepert, written for Adafruit Industries
#
# SPDX-License-Identifier: Unlicense
import time
import board
import math
import busio
from adafruit_bno08x import (
    BNO_REPORT_ACCELEROMETER,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_MAGNETOMETER,
    BNO_REPORT_ROTATION_VECTOR,
    BNO_REPORT_GAME_ROTATION_VECTOR,
    BNO_REPORT_GYROSCOPE,
    BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR
)
from adafruit_bno08x.i2c import BNO08X_I2C

i2c = busio.I2C(board.SCL, board.SDA)
bno = BNO08X_I2C(i2c)

# bno.enable_feature(BNO_REPORT_ACCELEROMETER)
# bno.enable_feature(BNO_REPORT_GYROSCOPE)
# bno.enable_feature(BNO_REPORT_MAGNETOMETER)
# bno.enable_feature(BNO_REPORT_ROTATION_VECTOR)
bno.enable_feature(BNO_REPORT_GAME_ROTATION_VECTOR)
# bno.enable_feature(BNO_REPORT_GYRO_INTEGRATED_ROTATION_VECTOR)

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

while True:
    time.sleep(0.1)
    # print("Acceleration:")
    # accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))
    # print("")

    # print("Gyro:")
    # gyro_x, gyro_y, gyro_z = bno.gyro  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f rads/s" % (gyro_x, gyro_y, gyro_z))
    # print("")

    # print("Magnetometer:")
    # mag_x, mag_y, mag_z = bno.magnetic  # pylint:disable=no-member
    # print("X: %0.6f  Y: %0.6f Z: %0.6f uT" % (mag_x, mag_y, mag_z))
    # print("")

    # print("Rotation Vector Quaternion:")
    # quat_i, quat_j, quat_k, quat_real = bno.game_quaternion  # pylint:disable=no-member
    # print(
    #     "I: %0.6f  J: %0.6f K: %0.6f  Real: %0.6f" % (quat_i, quat_j, quat_k, quat_real)
    # )
    # print("")
    # print('Game Quat')
    # degs = [c * 180/3.14159 for c in bno.game_quaternion]
    # print(degs)
    quat = bno.game_quaternion
    print('Game Euler')
    degs = [c * 180/3.14159 for c in euler_from_quaternion(quat[0], quat[1], quat[2], quat[3])]
    print(degs)


    # print('Gyro')
    # d = [c * 180/3.14159 for c in bno.gyro]

