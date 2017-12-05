#!/usr/bin/env python
import rospy
import web_interface
import time
from gcs.msg import waypoint, setPreflightData, deploy_request
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix
from std_msgs.msg import String

drone_number = 1
interface = web_interface.web_interface()
interface.setAuthentication('uasd','halogenlampe')

def missionDoneCallback(data):
    interface.setMissionDone(data.data)

def setPreflightDataCallback(data):
    drone_id = drone_number
    interface.setPreflightData(drone_id,data)

def setUavStateCallback(data):
    drone_id = drone_number
    interface.setUavState(drone_id,data.data)

def setUavCurrentLocationCallback(data):
    drone_id = drone_number
    interface.setUavCurrentLocation(drone_id,data)

def setPreflightDataCallback(data):
    drone_id = drone_number
    interface.setPreflightData(drone_id,data)

def main():
    ### Setup interface ###
    #interface = web_interface.web_interface()
    #interface.setAuthentication('uasd','halogenlampe')

    ### Setup ROS ###
    rospy.init_node('web_interface')
    deploy_request_publisher = rospy.Publisher('/web_interface/deploy_request',deploy_request, queue_size=1)
    request_id_publisher = rospy.Publisher('/web_interface/request_id', String, queue_size=1)

    mission_done_Subscriber = rospy.Subscriber('web_interface/listen/mission_done',Bool, missionDoneCallback)
    set_preflight_data_Subscriber = rospy.Subscriber('/web_interface/listen/set_preflight_data',setPreflightData,setPreflightDataCallback)
    set_uav_state_subscriber = rospy.Subscriber('/web_interface/listen/set_uav_state',String,setUavStateCallback)
    #set_uav_current_location = rospy.Subscriber('/web_interface/listen/set_uav_current_location',waypoint,setUavCurrentLocationCallback)
    set_uav_current_location = rospy.Subscriber('/drone_communication/globalPosition',NavSatFix,setUavCurrentLocationCallback)
    set_uav_preflight_data = rospy.Subscriber('/web_interface/listen/set_preflight_data',setPreflightData,setPreflightDataCallback)
    rate = rospy.Rate(1)
    rospy.loginfo("[web_interface] Node started.")
    while not rospy.is_shutdown():
        (update_count,lat,lng,alt, request_id) = interface.getDeployRequest()
        print 'Checking',update_count,lat,lng,alt
        if update_count == 0:
            msg = deploy_request()
            msg.point.lat = lat
            msg.point.lon = lng
            msg.point.alt = alt
            deploy_request_publisher.publish(msg)
            request_id_msg = String()
            request_id_msg.data = request_id
            request_id_publisher.publish(request_id_msg)
            print 'Published deploy request'
            rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: #Catches expctions to e.g. shutdown
        print 'Got exception'
        pass
