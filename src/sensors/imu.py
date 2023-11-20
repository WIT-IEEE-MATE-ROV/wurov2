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
