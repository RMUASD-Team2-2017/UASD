#!/usr/bin/env python
import rospy
import web_interface
from gcs.msg import waypoint, setPreflightData, deploy_request
from std_msgs.msg import Bool, String
from sensor_msgs.msg import NavSatFix

interface = web_interface.web_interface()
interface.setAuthentication('uasd','halogenlampe')

def missionDoneCallback(data):
    interface.setMissionDone(data.data)

def setPreflightDataCallback(data):
    drone_id = 1
    interface.setPreflightData(drone_id,data)

def setUavStateCallback(data):
    drone_id = 1
    interface.setUavState(drone_id,data.data)

def setUavCurrentLocationCallback(data):
    drone_id = 1
    interface.setUavCurrentLocation(drone_id,data)
def main():
    ### Setup interface ###
    #interface = web_interface.web_interface()
    #interface.setAuthentication('uasd','halogenlampe')

    ### Setup ROS ###
    rospy.init_node('web_interface')
    deploy_request_publisher = rospy.Publisher('/web_interface/deploy_request',deploy_request, queue_size=1)
    mission_done_Subscriber = rospy.Subscriber('web_interface/listen/mission_done',Bool, missionDoneCallback)
    set_preflight_data_Subscriber = rospy.Subscriber('/web_interface/listen/set_preflight_data',setPreflightData,setPreflightDataCallback)
    set_uav_state_subscriber = rospy.Subscriber('/web_interface/listen/set_uav_state',String,setUavStateCallback)
    #set_uav_current_location = rospy.Subscriber('/web_interface/listen/set_uav_current_location',waypoint,setUavCurrentLocationCallback)
    set_uav_current_location = rospy.Subscriber('/drone_communication/globalPosition',NavSatFix,setUavCurrentLocationCallback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        (update_count,lat,lng,alt) = interface.getDeployRequest()
        print 'Checking',update_count,lat,lng,alt
        if update_count == 0:
            msg = deploy_request()
            msg.point.lat = lat
            msg.point.lon = lng
            msg.point.alt = alt
            deploy_request_publisher.publish(msg)
            print 'Published deploy request'
            rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException: #Catches expctions to e.g. shutdown
        print 'Got exception'
        pass
