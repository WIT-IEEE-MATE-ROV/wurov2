#!/usr/bin/env python3

# By: Tedi Qafko
# Date: November 19, 2023

# Description:
# The imu.py script prints the 9 degrees of freedom data of the NXF9DOF module
# using a 3-axis gyroscope, 3-axis accelerometer, and a 3-axis magnetometer


# Libraries 
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3
import time
import board
import adafruit_fxas21002c 
import adafruit_fxos8700

# Setting up the NXF9DOF module I2C busses
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor1 = adafruit_fxas21002c.FXAS21002C(i2c)   # Gyroscope
sensor = adafruit_fxos8700.FXOS8700(i2c)        # Accelerometer and Magnetometer

while True:
    # Prints the 9-dof data of the imu in the terminal
    accel_x, accel_y, accel_z = sensor.accelerometer    # Units in m/s^2
    mag_x, mag_y, mag_z = sensor.magnetometer           # Units in Tesla
    gyro_x, gyro_y, gyro_z = sensor1.gyroscope          # Units in Radians/s

    print('Acceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (uTesla): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_x, mag_y, mag_z))
    print("Gyroscope (radians/s): ({0:0.3f},  {1:0.3f},  {2:0.3f})".format(gyro_x, gyro_y, gyro_z))

    time.sleep(0.1)



# import board
# import busio
# from adafruit_bno08x.i2c import BNO08X_I2C
# from adafruit_bno08x import BNO_REPORT_ACCELEROMETER

# i2c = busio.I2C(board.SCL, board.SDA)
# bno = BNO08X_I2C(i2c)
# bno.enable_feature(BNO_REPORT_ACCELEROMETER)

# while True:
#     accel_x, accel_y, accel_z = bno.acceleration  # pylint:disable=no-member
#     print("X: %0.6f  Y: %0.6f Z: %0.6f  m/s^2" % (accel_x, accel_y, accel_z))