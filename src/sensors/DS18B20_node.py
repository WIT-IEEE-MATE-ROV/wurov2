#!/usr/bin/env python3

import board
import Jetson.GPIO as GPIO

DS18B20_PIN = 7  # the pin that the temp sensor is pluged into

#BCM which refer to the pin number of the 40 pin GPIO header 
GPIO.setmode(GPIO.BOARD)
print(GPIO.getmode())
GPIO.setup(board.D12, GPIO.IN)





