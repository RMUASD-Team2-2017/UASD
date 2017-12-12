#!/usr/bin/env python
import rospy
import mavros
import json
import sensor_msgs
from gcs.srv import *
from std_msgs.msg import String
from std_msgs.msg import Bool
from weather import Weather
from mavros_msgs.msg import BatteryStatus
from sensor_msgs.msg import NavSatFix
from gcs.msg import dockData

num_battery_cells = 3

def batteryStatusSubscriberCallback(msg):
	global battery_voltage
	battery_voltage = float(msg.voltage)

def globalPositionSubscriberCallback(msg):
	global pos_latitude
	global pos_longitude
	pos_latitude = float(msg.latitude)
	pos_longitude = float(msg.longitude)

def dockDataSubscriberCallback(msg):
	global cell_voltage
	cell_voltage = msg.voltage

def gsmSubscriberCallback(msg):
	global gsm_ok
	decoded = json.loads(msg.data)
	conn_state = decoded['connectionState']
	gps_state = decoded['gpsState']
	#print "GSM content:",decoded['connectionState'],decoded['gpsState']
	gsm_ok = (conn_state == 1) & (gps_state == 2)
	#print gsm_ok
	# gsm_ok = conn_ok && gps_ok
	# The current state will not be updated if the link dies so we monitor the heartbeat as well
	gsm_ok = gsm_ok & gsm_heartbeat
	
def gsmHeartbeatSubscriberCallback(msg):
	global gsm_heartbeat
	gsm_heartbeat = msg.data
	#print gsm_heartbeat,gsm_ok

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

	for i in range(0, num_battery_cells - 1):
		rospy.loginfo(i)
		rospy.loginfo(cell_voltage[i])
		check = check & (cell_voltage[i] > 3.7)

	check = check & gsm_ok

	return preFlightResponse(gsm_ok, tmpt, hmd, wspd, battery_voltage, pos_latitude, pos_longitude)

def pre_flight():
	global battery_voltage
	battery_voltage = 0.0
	global cell_voltage
	cell_voltage = (0.0, 0.0, 0.0, 0.0);
	rospy.loginfo("[pre_flight] Cell voltages: %f %f %f %f", cell_voltage[0], cell_voltage[1], cell_voltage[2], cell_voltage[3])
	global gsm_ok
	gsm_ok = False
	global gsm_heartbeat
	gsm_heartbeat = False

	rospy.init_node('pre_flight')
	rospy.Subscriber("drone_communication/batteryStatus", BatteryStatus, batteryStatusSubscriberCallback)
	rospy.Subscriber("drone_communication/globalPosition", NavSatFix, globalPositionSubscriberCallback)
	rospy.Subscriber("docking_station/dock_data", dockData, dockDataSubscriberCallback)
	rospy.Subscriber("gsm_listener/fromDrone", String, gsmSubscriberCallback)
	rospy.Subscriber("gsm_listener/heartbeat", Bool, gsmHeartbeatSubscriberCallback)
	# weather = Weather()
	# weather.setLocation(55.471089000000006, 10.330159499999999, 1)
	# rospy.loginfo(weather.getPrecipitation())
	pfs = rospy.Service('pre_flight_node/preFlight', preFlight, handle_pre_flight_srv)
	rospy.loginfo("[pre_flight] Node started.")
	rospy.spin()

if __name__ == '__main__':
    try:
        pre_flight()
    except rospy.ROSInterruptException:
        pass
