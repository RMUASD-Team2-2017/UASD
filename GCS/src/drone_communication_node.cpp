//General

//custom

//ROS
#include <ros/ros.h>
#include "mavros_msgs/State.h"
#include "mavros_msgs/WaypointPush.h"
#include "mavros_msgs/CommandCode.h" // For CMD and NAV enums
#include "mavros_msgs/WaypointClear.h"
#include "mavros_msgs/CommandBool.h"
#include "mavros_msgs/CommandTOL.h"
#include "sensor_msgs/NavSatFix.h"
#include "mavros_msgs/CommandLong.h"


// start mission

#include "mavlink_defines.h"

#include "gcs/path.h"
#include "gcs/uploadMission.h"
#include "gcs/clearMission.h"
#include "gcs/arm.h"
#include "gcs/disarm.h"
#include "gcs/takeoff.h"
#include "gcs/land.h"
#include "gcs/startMission.h"
#include "gcs/stopMission.h"


// TODO Include mavlink to/from message types

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define DRONE_COMM_CLASS drone_comm

#define DEBUG true
#define HEARTBEAT_MAX_TIMEOUT 5 //Seconds

#define ROS_TAKEOFF 0
#define ROS_WAYPOINT 1
#define ROS_LAND 2

struct drone_status{
	sensor_msgs::NavSatFix gpsPosition;
	mavros_msgs::State state;
};

class DRONE_COMM_CLASS
{
public:
	DRONE_COMM_CLASS(ros::NodeHandle);
	~DRONE_COMM_CLASS();
	void checkHeartbeat();

private:
	// Methods


	// Attributes
	ros::NodeHandle nh;
	double heartbeatTimestamp;
	drone_status curDroneStatus;

	// Service servers
	ros::ServiceServer uploadMissionService;
	bool uploadMissionCallback(gcs::uploadMission::Request &req, gcs::uploadMission::Response &res);
	ros::ServiceServer clearMissionService;
	bool clearMissionCallback(gcs::clearMission::Request &req, gcs::clearMission::Response &res);
	ros::ServiceServer armService;
	bool armCallback(gcs::arm::Request &req, gcs::arm::Response &res);
	ros::ServiceServer disarmService;
	bool disarmCallback(gcs::disarm::Request &req, gcs::disarm::Response &res);
	ros::ServiceServer takeoffService;
	bool takeoffCallback(gcs::takeoff::Request &req, gcs::takeoff::Response &res);
	ros::ServiceServer landService;
	bool landCallback(gcs::land::Request &req, gcs::land::Response &res);
	ros::ServiceServer startMissionServer;
	bool startMissionCallback(gcs::startMission::Request &req, gcs::startMission::Response &res);
	ros::ServiceServer stopMissionServer;
	bool stopMissionCallback(gcs::stopMission::Request &req, gcs::stopMission::Response &res);

	// Service client
	ros::ServiceClient uploadMissionServiceClient;
	ros::ServiceClient clearMissionServiceClient;
	ros::ServiceClient armServiceClient;
	ros::ServiceClient takeoffServiceClient;
	ros::ServiceClient landServiceClient;
	ros::ServiceClient startMissionServiceClient;
	ros::ServiceClient stopMissionServiceClient;

	// Published topics

	// Subscribed topics
	ros::Subscriber stateSubscriber;
  void stateCallback(const mavros_msgs::State &msg);
	ros::Subscriber globalPositionSubscriber;
	void globalPositionCallback(const sensor_msgs::NavSatFix &msg);


	// TODO Implement service uploadWaypoints()

	// TODO Implement service monitorFlight()

};

DRONE_COMM_CLASS::DRONE_COMM_CLASS(ros::NodeHandle n)
{
	nh = n;
	// Services
	uploadMissionService = nh.advertiseService("/drone_communication/uploadMission",&DRONE_COMM_CLASS::uploadMissionCallback, this);
  clearMissionService = nh.advertiseService("/drone_communication/clearMission",&DRONE_COMM_CLASS::clearMissionCallback,this);
	armService = nh.advertiseService("/drone_communication/arm",&DRONE_COMM_CLASS::armCallback,this);
	disarmService = nh.advertiseService("/drone_communication/disarm",&DRONE_COMM_CLASS::disarmCallback,this);
	takeoffService = nh.advertiseService("/drone_communication/takeoff",&DRONE_COMM_CLASS::takeoffCallback,this);
	landService = nh.advertiseService("/drone_communication/land",&DRONE_COMM_CLASS::landCallback,this);
	startMissionServer = nh.advertiseService("/drone_communication/startMission",&DRONE_COMM_CLASS::startMissionCallback,this);
	stopMissionServer = nh.advertiseService("/drone_communication/stopMission",&DRONE_COMM_CLASS::stopMissionCallback,this);


	uploadMissionServiceClient = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros1/mission/push");
	clearMissionServiceClient = nh.serviceClient<mavros_msgs::WaypointClear>("/mavros1/mission/clear");
	armServiceClient = nh.serviceClient<mavros_msgs::CommandBool>("/mavros1/cmd/arming");
	takeoffServiceClient = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros1/cmd/takeoff");
	landServiceClient = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros1/cmd/land");
	startMissionServiceClient = nh.serviceClient<mavros_msgs::CommandLong>("/mavros1/cmd/Command");
	stopMissionServiceClient = nh.serviceClient<mavros_msgs::CommandLong>("/mavros1/cmd/Command");

	// Publishers
	// Subscribers
	stateSubscriber = nh.subscribe("/mavros1/state",1,&DRONE_COMM_CLASS::stateCallback,this);
	globalPositionSubscriber = nh.subscribe("/mavros1/global_position/global",1,&DRONE_COMM_CLASS::globalPositionCallback,this);

	heartbeatTimestamp = ros::Time::now().toSec();
}

DRONE_COMM_CLASS::~DRONE_COMM_CLASS()
{}

void DRONE_COMM_CLASS::stateCallback(const mavros_msgs::State &msg)
{
	heartbeatTimestamp = ros::Time::now().toSec();
	curDroneStatus.state = msg;
	if(DEBUG) ROS_INFO("Heartbeat");
}

void DRONE_COMM_CLASS::globalPositionCallback(const sensor_msgs::NavSatFix &msg)
{
	curDroneStatus.gpsPosition = msg;
	//if(DEBUG) ROS_INFO("Global position");
}

void DRONE_COMM_CLASS::checkHeartbeat()
{
	double now = ros::Time::now().toSec();
	if(now - heartbeatTimestamp > HEARTBEAT_MAX_TIMEOUT)
	{
		ROS_ERROR("Link lost %f %f", now, heartbeatTimestamp);
	}
}

bool DRONE_COMM_CLASS::uploadMissionCallback(gcs::uploadMission::Request &req, gcs::uploadMission::Response &res)
{
	if(DEBUG) ROS_INFO("Uploading mission");
	res.result = 0;
	int numberOfWaypoints = req.waypoints.path.size();
	if(DEBUG) ROS_INFO("Waypoints to upload: %i",numberOfWaypoints);
	mavros_msgs::WaypointPush missionMsg;
	for(int i=0; i<numberOfWaypoints; i++)
	{
		mavros_msgs::Waypoint wp;
		wp.frame = FRAME_GLOBAL_REL_ALT;
		switch (req.waypoints.path[i].type) {
			case ROS_TAKEOFF:
				wp.command = NAV_TAKEOFF;
				break;
			case ROS_WAYPOINT:
				wp.command = NAV_WAYPOINT;
				break;
			case ROS_LAND:
				wp.command = NAV_LAND;
				break;
			default:
				ROS_ERROR("WRONG COMMAND TYPE");
				break;
		}
		wp.is_current = false;
		wp.autocontinue = true;
		wp.x_lat = req.waypoints.path[i].lat;
		wp.y_long = req.waypoints.path[i].lon;
		wp.z_alt = req.waypoints.path[i].alt;
		missionMsg.request.waypoints.push_back(wp);
	}

/*
	mavros_msgs::WaypointPush missionMsg;
	mavros_msgs::Waypoint wp;
	wp.frame = FRAME_GLOBAL_REL_ALT;
	wp.command = NAV_TAKEOFF;
	wp.is_current = false;
	wp.autocontinue = true;
	wp.x_lat = 47.3977415;
	wp.y_long = 8.5455935;
	wp.z_alt = 487.0+5.0;
	missionMsg.request.waypoints.push_back(wp);
	//missionMsg.request.waypoints.push_back(wp);
*/
	if (uploadMissionServiceClient.call(missionMsg))
  {
    ROS_INFO("Uploded waypoints: %i",missionMsg.response.wp_transfered);
  }
  else
  {
    ROS_ERROR("Mission upload failed. Uploded waypoints: %i",missionMsg.response.wp_transfered);
  }
	return true;
}

bool DRONE_COMM_CLASS::clearMissionCallback(gcs::clearMission::Request &req, gcs::clearMission::Response &res)
{
	if(DEBUG) ROS_INFO("Clearing mission");
	mavros_msgs::WaypointClear msg;
	if(clearMissionServiceClient.call(msg))
	{
		if(msg.response.success == true)
		{
			res.result = 0;
			ROS_INFO("Waypoints cleared. Succes: %i Result %i", msg.response.success);
		}
		else
		{
			res.result = 1;
			ROS_ERROR("Failed to clear mission. Succes: %i Result %i", msg.response.success);
		}
	}
	else
	{
		ROS_ERROR("Failed to clear mission. Succes: %i Result %i", msg.response.success);
		res.result = 1;
	}
	return true;
}

bool DRONE_COMM_CLASS::armCallback(gcs::arm::Request &req, gcs::arm::Response &res)
{
	if(DEBUG) ROS_INFO("Arming");
	mavros_msgs::CommandBool msg;
	msg.request.value = true;
	if(armServiceClient.call(msg))
	{
		if(msg.response.success == true)
		{
			ROS_INFO("Armed. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
		else
		{
			ROS_ERROR("Failed to arm. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
	}
	else
	{
		ROS_ERROR("Failed to arm. Succes: %i Result %i", msg.response.success, msg.response.result);
	}
	return true;
}

bool DRONE_COMM_CLASS::disarmCallback(gcs::disarm::Request &req, gcs::disarm::Response &res)
{
	if(DEBUG) ROS_INFO("Disarming");
	mavros_msgs::CommandBool msg;
	msg.request.value = false;
	if(armServiceClient.call(msg))
	{
		if(msg.response.success == true)
		{
			ROS_INFO("Disarmed. Succes: %i Result %i", msg.response.success, msg.response.result);
			res.result = 0;
		}
		else
		{
			ROS_ERROR("Failed to disarm. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
	}
	else
	{
		ROS_ERROR("Failed to disarm. Succes: %i Result %i", msg.response.success, msg.response.result);
		res.result = 1;
	}
	return true;
}

bool DRONE_COMM_CLASS::takeoffCallback(gcs::takeoff::Request &req, gcs::takeoff::Response &res)
{
	// Add safety checks
	mavros_msgs::CommandTOL msg;
	msg.request.latitude = curDroneStatus.gpsPosition.latitude;
	msg.request.longitude = curDroneStatus.gpsPosition.longitude;
	msg.request.altitude = 10;
	if(takeoffServiceClient.call(msg))
	{
		if(msg.response.success == true)
		{
			if(DEBUG) ROS_INFO("Take off. Succes: %i Result %i", msg.response.success, msg.response.result);
			res.result = 0;
		}
		else
		{
			res.result = 1;
			ROS_ERROR("Failed to take off. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
	}
	else
	{
		res.result = 1;
		ROS_ERROR("Failed to takeoff. Succes: %i Result %i", msg.response.success, msg.response.result);
	}
}

bool DRONE_COMM_CLASS::landCallback(gcs::land::Request &req, gcs::land::Response &res)
{
	// Add safety checks
	mavros_msgs::CommandTOL msg;
	msg.request.latitude = curDroneStatus.gpsPosition.latitude;
	msg.request.longitude = curDroneStatus.gpsPosition.longitude;
	msg.request.altitude = 0;
	if(landServiceClient.call(msg))
	{
		if(msg.response.success == true)
		{
			if(DEBUG) ROS_INFO("Landed. Succes: %i Result %i", msg.response.success, msg.response.result);
			res.result = 0;
		}
		else
		{
			res.result = 1;
			ROS_ERROR("Failed to land. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
	}
	else
	{
		res.result = 1;
		ROS_ERROR("Failed to land. Succes: %i Result %i", msg.response.success, msg.response.result);
	}
}

bool DRONE_COMM_CLASS::startMissionCallback(gcs::startMission::Request &req, gcs::startMission::Response &res)
{

	mavros_msgs::CommandLong msg;
	msg.request.command = 300;
	msg.request.param1 = req.firstIndex;
	msg.request.param2 = req.lastIndex;
	if(startMissionServiceClient.call(msg))
	{
		if(msg.response.success == true)
		{
			if(DEBUG) ROS_INFO("Mission started. Succes: %i Result %i", msg.response.success, msg.response.result);
			res.result = 0;
		}
		else
		{
			res.result = 1;
			ROS_ERROR("Failed to start mission. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
	}
	else
	{
		res.result = 1;
		ROS_ERROR("Failed to start mission. Succes: %i Result %i", msg.response.success, msg.response.result);
	}
	return true;
}

bool DRONE_COMM_CLASS::stopMissionCallback(gcs::stopMission::Request &req, gcs::stopMission::Response &res)
{}

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(DRONE_COMM_CLASS));
	ros::NodeHandle nh;
	DRONE_COMM_CLASS dc(nh);
	while(ros::ok())
	{
		dc.checkHeartbeat();
		ros::spinOnce();
	}
  return 0;
}
