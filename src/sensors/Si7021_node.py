#!/usr/bin/env python3

import time
import board
import busio
import adafruit_si7021
import rospy
from std_msgs.msg import Float64

#

i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialize the SI7021.
sensor = adafruit_si7021.SI7021(i2c)

# Main loop to read the sensor values and print them every second.

def sensor_pub():
    e = rospy.init_node('si', anonymous=True)

    humidity_pub = rospy.Publisher('/si/humidity', Float64, queue_size=3)
    temperature_pub = rospy.Publisher('/si/temperature_c', Float64, queue_size=3)
    rate = rospy.Rate(1/ (20 / 1000))
    
    humid_data = Float64()
    temp_data = Float64()

    while not rospy.is_shutdown():
        humidity = sensor.relative_humidity # %
        temp = sensor.temperature # degree C

        humid_data.data = humidity # %
        temp_data.data = temp # degree C

        humidity_pub.publish(humid_data)
        temperature_pub.publish(temp_data)

        rate.sleep()


if __name__ == '__main__':
    sensor_pub()
