#!/usr/bin/env python3

import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField

import time
import board
import adafruit_fxas21002c
import adafruit_fxos8700

i2c = board.I2C()  # uses board.SCL and board.SDA
sensor1 = adafruit_fxas21002c.FXAS21002C(i2c) #Gyroscope
sensor = adafruit_fxos8700.FXOS8700(i2c) # Accelerometer and Magnetometer

# while True:
#     accel_x, accel_y, accel_z = sensor.accelerometer
#     mag_x, mag_y, mag_z = sensor.magnetometer
#     gyro_x, gyro_y, gyro_z = sensor1.gyroscope

#     print('Acceleration (m/s^2): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(accel_x, accel_y, accel_z))
#     print('Magnetometer (uTesla): ({0:0.3f}, {1:0.3f}, {2:0.3f})'.format(mag_x, mag_y, mag_z))
#     print("Gyroscope (radians/s): ({0:0.3f},  {1:0.3f},  {2:0.3f})".format(gyro_x, gyro_y, gyro_z))
#     time.sleep(0.1)

def imu_pub():
    # Initilizes a default ros node 
    rospy.init_node('imu_data_tedi', anonymous=True)

    pub_accel_data = rospy.Publisher('imu/data_raw', Imu, queue_size=5)
    pub_mag_data = rospy.Publisher('imu/mag', MagneticField, queue_size=5)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        accel_x, accel_y, accel_z = sensor.accelerometer
        mag_x, mag_y, mag_z = sensor.magnetometer
        gyro_x, gyro_y, gyro_z = sensor1.gyroscope
        
        data_raw = Imu()
        mag_raw = MagneticField()

        # Other
        data_raw.header.frame_id = None
        data_raw.header.stamp = rospy.Time.now()   
        mag_raw.header.stamp = rospy.Time.now() 
        # Imu angular velocity published to imu message type
        data_raw.angular_velocity.x = gyro_x
        data_raw.angular_velocity.y = gyro_y
        data_raw.angular_velocity.z = gyro_z
        # Imu linear acceleration published to imu message type
        data_raw.linear_acceleration.x = accel_x
        data_raw.linear_acceleration.y = accel_y
        data_raw.linear_acceleration.z = accel_z
        # Imu magnetometer data published to imu message type
        mag_raw.magnetic_field.x = mag_x
        mag_raw.magnetic_field.y = mag_y
        mag_raw.magnetic_field.z = mag_z

        pub_accel_data.publish(data_raw)
        pub_mag_data.publish(mag_raw)   

         
        rate.sleep()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    imu_pub()