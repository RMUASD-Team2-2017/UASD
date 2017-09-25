#!/usr/bin/env python
import rospy
from gcs.srv import *
from std_msgs.msg import String
from weather import Weather

def handle_pre_flight_srv(req):
	weather = Weather()
    # TODO get actual location
	weather.setLocation(55.471089000000006, 10.330159499999999, 1)
	check = float(weather.getWindSpeed()) < 10 # m/s
	is_wet = float(weather.getHumidity()) > 50 # Percent
	tmpt = float(weather.getTemperature()) # C
	
	if is_wet:
		check = check & ((tmpt > 0) & (tmpt < 40))
	else:
		check = check & ((tmpt > -10) & (tmpt < 40))
    # weather.getWindDirection()
    # weather.getWindCategory()

	return preFlightResponse(check)

def pre_flight():
	rospy.init_node('pre_flight')
	weather = Weather()
	weather.setLocation(55.471089000000006, 10.330159499999999, 1)
	rospy.loginfo(weather.getWindSpeed())
	pfs = rospy.Service('pre_flight_node/preflight', preFlight, handle_pre_flight_srv)
	rospy.spin()
	    
if __name__ == '__main__':
    try:
        pre_flight()
    except rospy.ROSInterruptException:
        pass
