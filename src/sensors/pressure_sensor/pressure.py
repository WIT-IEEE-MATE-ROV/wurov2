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
import MS5837
from std_msgs import Float32

# Setting up the MS5837 module I2C busses
i2c = board.I2C()  # uses board.SCL and board.SDA
sensor = ms5837.MS5837_30BA(i2c) 

def pres_pub():
    
    rospy.init_node('ms5837_data', anonymous=True)

    pressure = rospy.Publisher('ms5837/pressure', Float32, queue_size=5)
    temperature = rospy.Publisher('ms5837/temperature', Float32, queue_size=5)
    depth = rospy.Publisher('ms5837/depth', Float32, queue_size=5)

    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        pres_raw = Float32()      
        temp_raw = Float32() 
        depth_raw = Float32()

        pres_raw.data = sensor.pressure() # ms5837.UNITS_Torr | ms5837.UNITS_psi | ms5837.UNITS_atm default mbar
        temp_raw.data = sensor.temperature() # ms5837.UNITS_Farenheit | ms5837.UNITS_Kelvin | ms5837.UNITS_Centigrade default Celcius
        depth_raw = sensor.depth()
        # sensor.setFluidDensity(ms5837.DENSITY_SALTWATER)
        # saltwaterDepth = sensor.depth() # No nead to read() again
        # sensor.setFluidDensity(1000) # kg/m^3

        pressure.publish(data_raw)
        temperature.publish(mag_raw)   
        depth.publish(depth_raw)

        rate.sleep()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    pres_pub()