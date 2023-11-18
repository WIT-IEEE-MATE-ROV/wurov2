"""
Python code for contolling the PCA9685

"""
import rospy
from std_msgs.msg import UInt16
from geometry_msgs.msg import Vector3
import time
from board import *
import asyncio
import busio
import smbus
import Jetson.GPIO as GPIO

# from smbus2 import SMBus
#GPIO.setmode(GPIO.BOARD)

# SDA_PIN = GPIO.setup(3, GPIO.OUT, initial=GPIO.HIGH)
# SCL_PIN = GPIO.setup(5, GPIO.OUT, initial=GPIO.HIGH)

# regster num
PCA_REG_PRE_SCALE = 0xFE
PCA_REG_MODE_1 = 0x00

# map for mode 1
PCA_M1_RESTART = 1 << 7
PCA_M1_EXTCLK = 1 << 6
PCA_M1_AUTO_INC = 1 << 5
PCA_M1_SLEEP = 1 << 4
PCA_CTRL_REG_OFFSET = 0x06


class PCA9685():
    
    def __init__(self, i2c_address:int, frequency_hz:float, measured_frequency = None):
        self.i2c_address = i2c_address
        self.__i2c_bus = busio.I2C(SCL, SDA)
        self.__sbus =   

        if measured_frequency is not None:
            self.__measured_frequency = measured_frequency
        else:
            self.__measured_frequency = frequency_hz
        
        self.__frequency_hz = frequency_hz


    def reg_read(self, reg, result):
        while not self.__i2c_bus.try_lock():
            pass
        try:
            self.__i2c_bus.writeto_then_readfrom(self.i2c_address, bytes([reg]), result, in_start=0, out_start=0) 
            self.__i2c_bus.unlock()
              
            return result
        except Exception as e:
            self.__i2c_bus.unlock()
            raise e
            

    def reg_write(self, reg, data):
        while not self.__i2c_bus.try_lock():
            pass
        try:
            buf = bytearray(1)
            buf[0] = reg
            buf.extend(data)
            self.__i2c_bus.writeto(self.i2c_address, buf)
            self.__i2c_bus.unlock()
        except Exception as e:
            self.__i2c_bus.unlock()
            raise e


    def set_measured_frequency(self, measured_frequency:float):
        self.__measured_frequency = measured_frequency


    def restart(self):
        buf = bytearray(1)
        self.reg_read(PCA_REG_MODE_1, buf)
        buf[0] |= PCA_M1_RESTART
        self.reg_write(PCA_REG_MODE_1 ,buf)
        

    # def set_sleep(self, sleep_on:bool):
    #     mode_1 = bytearray(1)
    #     print("about to read in sleep")
    #     self.reg_read(PCA_REG_MODE_1, mode_1)
    #     sleep_state = mode_1[0] & PCA_M1_SLEEP
    #     if sleep_on and not sleep_state:
    #         mode_1[0] |= PCA_M1_SLEEP
    #         print(mode_1[0])
    #         self.reg_write(PCA_REG_MODE_1, mode_1)
    #     elif not sleep_on and sleep_state:
    #         mode_1[0] &= ~PCA_M1_SLEEP
    #         print(mode_1[0])
    #         self.reg_write(PCA_REG_MODE_1, mode_1)



    def setup(self):
        ... 


p = PCA9685(0x40, 100)
w = [0]
print("before", w)
print(p.reg_read(PCA_REG_MODE_1, w))
print("after", w)

print(p.reg_write(PCA_REG_MODE_1, bytes([PCA_M1_AUTO_INC])))
print(p.reg_read(PCA_REG_MODE_1, w))
print("after", w)
