#!/usr/bin/env python3

import rospy
import numpy as np
from brping import Ping360
from sensor_msgs.msg import LaserScan
import math
import json
from rospy_message_converter import message_converter

class ping360:
    def __init__(self):
        """
        ping360 node is used to get temperature and depth readings from a ping360 sensor
        duration is taken from the launch file param server
        """
        
        rospy.init_node('temperature', anonymous=True)

        self._publisher = rospy.Publisher('vehicle/ping360', LaserScan, queue_size=3)

        # publishDuration = rospy.get_param(f"/{rospy.get_name()}/rate")
        # interface = rospy.get_param(f"/{rospy.get_name()}/interface")
        # baudrate = rospy.get_param(f"/{rospy.get_name()}/baudrate")

        self.sensor = Ping360()
        # self.sensor.connect_serial(interface, baudrate)
        # self.sensor.initialize()

        # rospy.Timer(rospy.Duration(publishDuration), self.publisher)

        rospy.spin()

    def publisher(self, data):
        msg = LaserScan()
        msg.angle_min = 0
        msg.angle_max = 6.28319 #360 degrees
        msg.angle_increment = 0.314159 #20 gradians

        msg.ranges = []
        data = []
        for val in range(20):
            scan = self.sensor.transmitAngle(val * 20) #angle in gradians
            data = scan.data

            #NOTE: This is temp for debugging, stores raw data in sonar_raw.json
            with open("sonar_raw.json", "r") as file:
                raw_data = json.load(file)

            raw_data.append(raw_data)

            with open("sonar_raw.json", "w") as file:
                json.dump(raw_data, file)

            msg.ranges.append(-1)
            for detectedIntensity in data:
                if detectedIntensity >= 200:
                    detectedIndex = data.index(detectedIntensity)
                    rangeVal = (1+detectedIndex)  * 1481 * 25e-9 * scan.sample_period/ 2
                    if rangeVal >= 0.75:
                        msg.ranges.pop()
                        msg.ranges.append(rangeVal)
                        break
            
        self._publisher.publish(msg)

        #NOTE: This is temp for debugging, stores sonar msg as json for later
        with open("sonar.json", "r") as file:
            msg_data = json.load(file)

        msg_data.append(message_converter.convert_ros_message_to_dictionary(msg))

        with open("sonar.json", "w") as file:
            json.dump(msg_data, file)

        self._publisher.publish(msg)

if __name__ == '__main__':
    ping360() 