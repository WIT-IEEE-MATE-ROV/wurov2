#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3

import time
import board
import adafruit_fxas21002c
import adafruit_fxos8700

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor1 = adafruit_fxas21002c.FXAS21002C(i2c) #Gyroscope
sensor = adafruit_fxos8700.FXOS8700(i2c) # Accelerometer and Magnetometer

while True:
    accel_x, accel_y, accel_z = sensor.accelerometer
    mag_x, mag_y, mag_z = sensor.magnetometer
    gyro_x, gyro_y, gyro_z = sensor1.gyroscope

    print('Acceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z))
    print('Magnetometer (uTesla): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_x, mag_y, mag_z))
    print("Gyroscope (radians/s): ({0:0.3f},  {1:0.3f},  {2:0.3f})".format(gyro_x, gyro_y, gyro_z))
    time.sleep(0.1)
