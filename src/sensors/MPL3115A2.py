#!/usr/bin/env python3

import time
import board
import busio
import adafruit_mpl3115a2
import rospy
from std_msgs.msg import Float64



i2c = board.I2C()  # uses board.SCL and board.SDA
# i2c = board.STEMMA_I2C()  # For using the built-in STEMMA QT connector on a microcontroller

# Initialize the MPL3115A2.
sensor = adafruit_mpl3115a2.MPL3115A2(i2c)

sensor.sealevel_pressure = 1025.7

# Main loop to read the sensor values and print them every second.

def sensor_pub():
    e = rospy.init_node('mpl', anonymous=True)

    pressure_pub = rospy.Publisher('/mpl/pressure_hPa', Float64, queue_size=3)
    altitude_pub = rospy.Publisher('/mpl/altitude_m', Float64, queue_size=3)
    temperature_pub = rospy.Publisher('/mpl/temperature_f', Float64, queue_size=3)
    rate = rospy.Rate(1/ (20 / 1000))
    
    pres_data = Float64()
    alt_data = Float64()
    temp_data = Float64()

    while not rospy.is_shutdown():
        pressure = sensor.pressure
        # print("Pressure: {0:0.3f} hectopascals".format(pressure))
        altitude = sensor.altitude
        # print("Altitude: {0:0.3f} meters".format(altitude))
        temperature = sensor.temperature
        fahr = temperature * (9/5) + 32
        # print("Temperature: {0:0.3f} Celsius".format(temperature))
        # print(f'Freedom Temperature: {temperature * (9/5) + 32} Fahrenheit')
        pres_data.data = pressure
        alt_data.data = altitude
        temp_data.data = fahr

        pressure_pub.publish(pres_data)
        altitude_pub.publish(alt_data)
        temperature_pub.publish(temp_data)

        rate.sleep()


if __name__ == '__main__':
    sensor_pub()