#!/usr/bin/env python
import rospy
import mavros
import sensor_msgs
from gcs.srv import *
from std_msgs.msg import String
from weather import Weather
from mavros_msgs.msg import BatteryStatus
from sensor_msgs.msg import NavSatFix

num_battery_cells = 3

def batteryStatusSubscriberCallback(msg):
	global battery_voltage
	battery_voltage = float(msg.voltage)

def globalPositionSubscriberCallback(msg):
	global pos_latitude
	global pos_longitude
	pos_latitude = float(msg.latitude)
	pos_longitude = float(msg.longitude)

def handle_pre_flight_srv(req):
	weather = Weather()
	weather.setLocation(pos_latitude, pos_longitude, 1)
	wspd = float(weather.getWindSpeed())
	check = float(wspd) < 10 # m/s
	hmd = float(weather.getHumidity())
	is_wet = float(hmd) > 50 # Percent
	tmpt = float(weather.getTemperature()) # C

	if is_wet:
		check = check & ((tmpt > 0) & (tmpt < 40))
	else:
		check = check & ((tmpt > -10) & (tmpt < 40))
    # weather.getWindDirection()
    # weather.getWindCategory()

	battery_threshold = num_battery_cells * 3.7
	check = check & (battery_voltage > battery_threshold)

	return preFlightResponse(check, tmpt, hmd, wspd, battery_voltage, pos_latitude, pos_longitude)

def pre_flight():
	global battery_voltage
	battery_voltage = 0.0

	rospy.init_node('pre_flight')
	rospy.Subscriber("drone_communication/batteryStatus", BatteryStatus, batteryStatusSubscriberCallback)
	rospy.Subscriber("drone_communication/globalPosition", NavSatFix, globalPositionSubscriberCallback)
	# weather = Weather()
	# weather.setLocation(55.471089000000006, 10.330159499999999, 1)
	# rospy.loginfo(weather.getWindSpeed())
	pfs = rospy.Service('pre_flight_node/preFlight', preFlight, handle_pre_flight_srv)
	rospy.spin()

if __name__ == '__main__':
    try:
        pre_flight()
    except rospy.ROSInterruptException:
        pass
