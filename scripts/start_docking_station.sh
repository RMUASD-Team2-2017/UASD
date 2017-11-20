#!/bin/bash
export ROS_MASTER_URI=http://192.168.1.2:11311
export ROS_IP=192.168.1.11  
source /home/pi/catkin_ws/devel/setup.bash
rosrun gcs docking_station_node.py
