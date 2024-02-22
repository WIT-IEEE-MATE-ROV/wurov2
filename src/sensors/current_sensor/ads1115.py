#!/usr/bin/env python3

# By: Tedi Qafko
# Date: Feb 19, 2024

# Description: ADC turns analog voltages to digital sensors. 
# This library focuses in the current sensors ACS724

# Datasheets:
# https://www.digikey.com/en/products/detail/allegro-microsystems/ACS724LLCTR-10AB-T/5175288 
# https://cdn-shop.adafruit.com/datasheets/ads1115.pdf 

# Libraries 
import rospy
import time
import board
import adafruit_ads1x15.ads1115 as ADS
from adafruit_ads1x15.analog_in import AnalogIn
from std_msgs import Float32

# Setting up the ADS1115 module I2C busses
i2c = board.I2C()  # uses board.SCL and board.SDA

ads = ADS.ADS1115(i2c)
ads.mode = Mode.SINGLE # allows for multiple channels, it tells sensor to wait for ADS to complete conversion

# Channels where data is saved from pins
chan_5V = AnalogIn(ads, ADS.P0) # TODO: Temporary P0 for now change channel when we know 
chan_12V = AnalogIn(ads, ADS.P1) # TODO: Temporary P1 for now change channel when we know

def ads_pub():
    rospy.init_node('ads1115_data', anonymous=True)
    current_5V = rospy.Publisher('ads1115/current_5V', Float32, queue_size=5)
    current_12V = rospy.Publisher('ads1115/current_12V', Float32, queue_size=5)
    rate = rospy.Rate(10) # 10hz

    while not rospy.is_shutdown():
        chan_5V_voltage_raw = Float32()      
        chan_12V_voltage_raw = Float32()

        # TODO: Turn raw data to current readings, these are volts now 
        chan_5V_voltage_raw.data = chan_5V.value 
        chan_12V_voltage_raw.data = chan_12V.value
        
        # TODO: Test voltages out first, need to turn these to current values Amps
        current_5V.publish(chan_5V_voltage_raw)
        current_12V.publish(chan_12V_voltage_raw)  
        rate.sleep()
    
    # spin() simply keeps python from exiting until this node is stopped
    rospy.spin()

if __name__ == '__main__':
    ads_pub()