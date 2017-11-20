#!/usr/bin/env python
import rospy
from gcs.srv import *
from gcs.msg import dockData
import sys 
import RPi.GPIO as GPIO  
import Adafruit_DHT

pin=2

def handle_openDock_srv(req):
    return open_docking_station()

def open_docking_station():
    # Do some GPIO stuff here to open the docking station
    return True

def docking_station():
    rospy.init_node("docking_station")
    s = rospy.Service("/docking_station/openDock", openDock, handle_openDock_srv)
    publish_data()
    rospy.spin()

def publish_data():
    pub = rospy.Publisher("/docking_station/dock_data", dockData, queue_size=1)
    rate = rospy.Rate(2000) # 10hz
    data = dockData()
    
    while not rospy.is_shutdown():
        humidity, temperature = Adafruit_DHT.read_retry(Adafruit_DHT.DHT22, pin)  # Read temperature and humidity from sensor

        if humidity is not None and temperature is not None:
            data.temperature = temperature  # degree celcius
            data.humidity = humidity  # relative: 0-100%
            pub.publish(data)   
        else:
            print "Failed to get temperature and humidity reading, will try again!"
        
        rate.sleep()
    
   
if __name__ == "__main__":
    try:
        docking_station()
    except rospy.ROSInterruptException:
        pass

