#!/usr/bin/env python
import rospy
import web_interface
from gcs.msg import waypoint
from std_msgs.msg import Bool

interface = web_interface.web_interface()
interface.setAuthentication('uasd','halogenlampe')

def mission_mission_done_callback(data):
    interface.setMissionDone(data.data)


def test():
    ### Setup interface ###
    #interface = web_interface.web_interface()
    #interface.setAuthentication('uasd','halogenlampe')

    ### Setup ROS ###
    rospy.init_node('web_interface')
    deploy_request_publisher = rospy.Publisher('/web_interface/deploy_request',waypoint, queue_size=1)
    mission_done_Subscriber = rospy.Subscriber('web_interface/listen/mission_done',Bool, mission_mission_done_callback)

    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        (update_count,lat,lng,alt) = interface.getDeployRequest()
        print 'Checking',update_count,lat,lng,alt
        if update_count == 0:
            msg = waypoint()
            msg.lat = lat
            msg.lon = lng
            msg.alt = alt
            deploy_request_publisher.publish(msg)
            print 'Published deploy request'
            rospy.loginfo(msg)
        rate.sleep()

if __name__ == '__main__':
    try:
        test()
    except rospy.ROSInterruptException: #Catches expctions to e.g. shutdown
        print 'Got exception'
        pass
