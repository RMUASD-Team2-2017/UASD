#!/usr/bin/env python
import rospy
from gcs.srv import *
from std_msgs.msg import String
from weather import Weather

def handle_pre_flight_srv():
    return preFlightResponse()

def pre_flight():
    rospy.init_node('pre_flight')
    weather = Weather();
    # TODO get actual location
    weather.setLocation(55.471089000000006, 10.330159499999999, 1) 
    pfs = rospy.Service('pre_flight_srv', preFlight, handle_pre_flight_srv)
    rospy.spin()
	    
if __name__ == '__main__':
    try:
        pre_flight()
    except rospy.ROSInterruptException:
        pass
