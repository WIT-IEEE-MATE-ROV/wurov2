#!/usr/bin/env bash

# This scropt is used to set up a remote ssh connection to the robot
# for beginning a node using launch files in ros
sudo chmod a+rw /dev/i2c-*
source /opt/ros/noetic/setup.bash
source catkin_ws/devel/setup.bash
export ROS_MASTER_URI=http://192.168.1.2:11311/
export ROS_IP="192.168.1.1"

exec "$@"
