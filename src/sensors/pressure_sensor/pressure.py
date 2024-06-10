#!/usr/bin/env python3

# By: Tedi Qafko
# Date: Feb 19, 2024

# Description:
# Sensor MS5837 retrived pressure and temperature as well as converters 
# the data to depth in water.

# Libraries 
import rospy
import time
import board
from ms5837 import MS5837_02BA, UNITS_Farenheit
from std_msgs.msg import Float32

# Setting up the MS5837 module I2C busses
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = MS5837_02BA(1)

def pres_pub():
    
    rospy.init_node('ms5837_data', anonymous=True)

    pressure = rospy.Publisher('ms5837/pressure', Float32, queue_size=5)
    temperature = rospy.Publisher('ms5837/temperature', Float32, queue_size=5)
    depth = rospy.Publisher('ms5837/depth', Float32, queue_size=5)

    pres_raw = Float32()      
    temp_raw = Float32() 
    depth_raw = Float32()
    sensor.init()
    sensor.setFluidDensity(1000)# kg/m^3

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        try:
            sensor.read()

            # pres_raw.data = sensor.pressure() # ms5837.UNITS_Torr | ms5837.UNITS_psi | ms5837.UNITS_atm default mbar
            temp_raw.data = sensor.temperature(UNITS_Farenheit) # ms5837.UNITS_Farenheit | ms5837.UNITS_Kelvin | ms5837.UNITS_Centigrade default Celcius
            depth_raw.data = sensor.depth() # m
        except IOError as ioe:
            print('I2C Error')

        pressure.publish(pres_raw)
        temperature.publish(temp_raw)   
        depth.publish(depth_raw)

        rate.sleep()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pres_pub()