#!/usr/bin/env python3

# By: Tedi Qafko
# Date: November 19, 2023

# Description:
# Publish 9dof data from the NXF9DOF module to the topics /imu/data_raw and /imu/mag


# Libraries 
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3
from sensor_msgs.msg import Imu
from sensor_msgs.msg import MagneticField
import time
import board
import adafruit_fxas21002c
import adafruit_fxos8700

# Setting up the NXF9DOF module I2C busses
i2c = board.I2C()  # uses board.SCL and board.SDA
gyro = adafruit_fxas21002c.FXAS21002C(i2c) #Gyroscope
accel_mag = adafruit_fxos8700.FXOS8700(i2c) # Accelerometer and Magnetometer

# Offset Values were calculated from the 9dof_calibration script
mag_offset = [19.050000000000004, -21.950000000000003, -41.900000000000006]
gyro_offset = [-0.025293638769136574, -0.04970097752749478, -0.030475084985994737]

def imu_pub():
    # Begins a node that publishes the data from the NXF9DOF module to topics.
    rospy.init_node('imu_data', anonymous=True)
    pub_accel_data = rospy.Publisher('imu/data_raw', Imu, queue_size=5)
    pub_mag_data = rospy.Publisher('imu/mag', MagneticField, queue_size=5)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        # Retrives and publishes the 9-dof data to the imu topics
        accel_x, accel_y, accel_z = accel_mag.accelerometer # Units in m/s^2
        mag_x, mag_y, mag_z = accel_mag.magnetometer        # Units in Tesla
        gyro_x, gyro_y, gyro_z = gyro.gyroscope             # Units in Radians/s
        
        data_raw = Imu()            # An Imu data type msg
        mag_raw = MagneticField()   # A MagneticField data type msg

        # Header data needed in the imu and magneticfield data types
        data_raw.header.frame_id = None
        data_raw.header.stamp = rospy.Time.now()   
        mag_raw.header.stamp = rospy.Time.now() 
        # Angular velocity saved from nxf module to the imu variable
        data_raw.angular_velocity.x = gyro_x - gyro_offset[0]
        data_raw.angular_velocity.y = gyro_y - gyro_offset[1]
        data_raw.angular_velocity.z = gyro_z - gyro_offset[2]
        # Linear acceleration saved from nxf module to the imu variable
        data_raw.linear_acceleration.x = accel_x
        data_raw.linear_acceleration.y = accel_y
        data_raw.linear_acceleration.z = accel_z
        # Magnetometer saved from nxf module to the imu variable
        mag_raw.magnetic_field.x = mag_x - mag_offset[0]
        mag_raw.magnetic_field.y = mag_y - mag_offset[1]
        mag_raw.magnetic_field.z = mag_z - mag_offset[2]
        # Imu and magnetic field types publish to the topics
        pub_accel_data.publish(data_raw)
        pub_mag_data.publish(mag_raw)   
        rate.sleep()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    imu_pub()