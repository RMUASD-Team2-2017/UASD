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
#include "mavros_msgs/ParamSet.h"
#include "mavros_msgs/SetMode.h"
#include "mavros_msgs/WaypointList.h"
#include "mavros_msgs/BatteryStatus.h"

#include "mavlink_defines.h"	// For mavlink defines not accessible in other files

#include "gcs/path.h"
#include "gcs/uploadMission.h"
#include "gcs/clearMission.h"
#include "gcs/arm.h"
#include "gcs/disarm.h"
#include "gcs/takeoff.h"
#include "gcs/land.h"
#include "gcs/startMission.h"
#include "gcs/stopMission.h"
#include "gcs/communicationStatus.h"



// TODO Include mavlink to/from message types

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define DRONE_COMM_CLASS drone_comm

#define DEBUG true
#define HEARTBEAT_MAX_TIMEOUT 5 //Seconds

#define HOLD_TIME 				0.0
#define ACCEPTANCE_RADIUS		1.0

#define ROS_TAKEOFF 0
#define ROS_WAYPOINT 1
#define ROS_LAND 2

#define SUCCESS					0
#define ERROR_CLEARING_MISSION 	1
#define ERROR_PARAM_SET 		2
#define ERROR_SET_MODE			3
#define ERROR_UPLOAD_MISSION	4
#define ERROR					5
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
	void sendSetup();


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
	ros::ServiceClient setParameterServiceClient;
	ros::ServiceClient setModeServiceClient;

	// Published topics
	ros::Publisher communicationStatusPublisher;
	ros::Publisher dronePositionPublisher;
	ros::Publisher statePublisher;
	ros::Publisher missionStatusPublisher;
	ros::Publisher batteryStatusPublisher;

	// Subscribed topics
	ros::Subscriber stateSubscriber;
  	void stateCallback(const mavros_msgs::State &msg);
	ros::Subscriber globalPositionSubscriber;
	void globalPositionCallback(const sensor_msgs::NavSatFix &msg);
	ros::Subscriber missionStatusSubscriber;
	void missionStatusSubscriberCallback(const mavros_msgs::WaypointList &msg);
	ros::Subscriber batteryStatusSubscriber;
	void batteryStatusSubscriberCallback(const mavros_msgs::BatteryStatus &msg);
	
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

	// Service clients
	uploadMissionServiceClient = nh.serviceClient<mavros_msgs::WaypointPush>("/mavros1/mission/push");
	clearMissionServiceClient = nh.serviceClient<mavros_msgs::WaypointClear>("/mavros1/mission/clear");
	armServiceClient = nh.serviceClient<mavros_msgs::CommandBool>("/mavros1/cmd/arming");
	takeoffServiceClient = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros1/cmd/takeoff");
	landServiceClient = nh.serviceClient<mavros_msgs::CommandTOL>("/mavros1/cmd/land");
	startMissionServiceClient = nh.serviceClient<mavros_msgs::CommandLong>("/mavros1/cmd/Command");
	stopMissionServiceClient = nh.serviceClient<mavros_msgs::CommandLong>("/mavros1/cmd/Command");
	setParameterServiceClient = nh.serviceClient<mavros_msgs::ParamSet>("/mavros1/param/set");
	setModeServiceClient = nh.serviceClient<mavros_msgs::SetMode>("/mavros1/set_mode");

	// Publishers
	communicationStatusPublisher = nh.advertise<gcs::communicationStatus>("/drone_communication/communiationStatus",1);
	dronePositionPublisher = nh.advertise<sensor_msgs::NavSatFix>("/drone_communication/globalPosition",1);
	statePublisher = nh.advertise<mavros_msgs::State>("/drone_communication/droneState",1);
	missionStatusPublisher = nh.advertise<mavros_msgs::WaypointList>("/drone_communication/missionState",1);
	batteryStatusPublisher = nh.advertise<mavros_msgs::BatteryStatus>("drone_communication/batteryStatus",1);
	// Subscribers
	stateSubscriber = nh.subscribe("/mavros1/state",1,&DRONE_COMM_CLASS::stateCallback,this);
	globalPositionSubscriber = nh.subscribe("/mavros1/global_position/global",1,&DRONE_COMM_CLASS::globalPositionCallback,this);
	missionStatusSubscriber = nh.subscribe("/mavros1/mission/waypoints",1,&DRONE_COMM_CLASS::missionStatusSubscriberCallback, this);
	batteryStatusSubscriber = nh.subscribe("/mavros1/battery",1,&DRONE_COMM_CLASS::batteryStatusSubscriberCallback,this);

	heartbeatTimestamp = ros::Time::now().toSec();
}

DRONE_COMM_CLASS::~DRONE_COMM_CLASS()
{}

void DRONE_COMM_CLASS::stateCallback(const mavros_msgs::State &msg)
{
	heartbeatTimestamp = ros::Time::now().toSec();
	curDroneStatus.state = msg;
	statePublisher.publish(msg);
	if(DEBUG) ROS_INFO("Heartbeat");
}

void DRONE_COMM_CLASS::globalPositionCallback(const sensor_msgs::NavSatFix &msg)
{
	curDroneStatus.gpsPosition = msg;
	dronePositionPublisher.publish(msg);
	//if(DEBUG) ROS_INFO("Global position");
}

void DRONE_COMM_CLASS::missionStatusSubscriberCallback(const mavros_msgs::WaypointList &msg)
{
	missionStatusPublisher.publish(msg);
}

void DRONE_COMM_CLASS::batteryStatusSubscriberCallback(const mavros_msgs::BatteryStatus &msg)
{
	batteryStatusPublisher.publish(msg);
}

void DRONE_COMM_CLASS::checkHeartbeat()
{
	double now = ros::Time::now().toSec();
	if(now - heartbeatTimestamp > HEARTBEAT_MAX_TIMEOUT)
	{
		gcs::communicationStatus statusMsg;
		statusMsg.connected = false;
		communicationStatusPublisher.publish(statusMsg);
		//ROS_ERROR("Link lost %f %f", now, heartbeatTimestamp);
	}

}

bool DRONE_COMM_CLASS::uploadMissionCallback(gcs::uploadMission::Request &req, gcs::uploadMission::Response &res)
{
	if(DEBUG) ROS_INFO("Uploading mission");
	res.result = SUCCESS;
	mavros_msgs::WaypointClear clearMsg;
	while(true)
	{
		if(!clearMissionServiceClient.call(clearMsg) && !clearMsg.response.success)
		{
			ROS_ERROR("Error in clearing mission");
			res.result = ERROR_CLEARING_MISSION;
			return false;
		}
		else
			break;
	}
	mavros_msgs::ParamSet paramMsg;
	paramMsg.request.param_id = "NAV_DLL_ACT";
	while(true)
	{
		if(!setParameterServiceClient.call(paramMsg) && !paramMsg.response.success)
		{
			ROS_ERROR("Error in setting NAV_DLL_ACT");
			res.result = ERROR_PARAM_SET;
			return false;
		}
		else
			break;
	}
	mavros_msgs::SetMode setModeMsg;
	setModeMsg.request.custom_mode = "AUTO.MISSION";
	while(true)
	{
		if(!setModeServiceClient.call(setModeMsg) && !setModeMsg.response.success)
		{
			ROS_ERROR("Error in setting mode AUTO.MISSION");
			res.result = ERROR_SET_MODE;
			return false;
		}
		else
			break;
	}

	int numberOfWaypoints = req.waypoints.path.size();
	if(DEBUG) ROS_INFO("Waypoints to upload: %i",numberOfWaypoints);
	mavros_msgs::WaypointPush missionMsg;
	for(int i=0; i<numberOfWaypoints; i++)
	{
		ROS_INFO("Wp: %i lat: %f lon: %f alt: %f type: %i", i, req.waypoints.path[i].lat , req.waypoints.path[i].lon, req.waypoints.path[i].alt, req.waypoints.path[i].type);
		mavros_msgs::Waypoint wp;
		wp.frame = FRAME_GLOBAL_REL_ALT;
		switch (req.waypoints.path[i].type) {
			case ROS_TAKEOFF:
				wp.command = NAV_TAKEOFF;
				// param1 minimum desired pitch (only fixed wing)
				// Param2 empty
				// Param3 empty
				// param4 yaw, NAN for unchaged
				break;
			case ROS_WAYPOINT:
				wp.command = NAV_WAYPOINT;
				wp.param1 = HOLD_TIME;
				wp.param2 = ACCEPTANCE_RADIUS;
				wp.param3 = 0;		// Pass through 0, radius [m] cc > 0, radius [m] ccv < 0
				wp.param4 = 0; 		// Yaw angle
				break;
			case ROS_LAND:
				wp.command = NAV_LAND;
				// param1 abort altitude. Altitude to climb to if landing is aborted
				// param2 empty
				// param3 empty
				// param4 yaw. NaN for unchanged
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

	// mavros_msgs::WaypointPush missionMsg;
	// mavros_msgs::Waypoint wp;
	// wp.frame = FRAME_GLOBAL_REL_ALT;
	// wp.command = NAV_TAKEOFF;
	// wp.is_current = false;
	// wp.autocontinue = true;
	// wp.x_lat = 47.3977415;
	// wp.y_long = 8.5455943;
	// wp.z_alt = 5.0;
	// missionMsg.request.waypoints.push_back(wp);

	// mavros_msgs::Waypoint wp1;
	// wp1.frame = FRAME_GLOBAL_REL_ALT;
	// wp1.command = NAV_WAYPOINT;
	// wp1.is_current = false;
	// wp1.autocontinue = true;
	// wp1.param1 = 5.0;				// Hold time [decimal seconds]
	// wp1.param2 = 1.0;				// Acceptance radius [m]
	// wp1.param3 = 0.0;				// Pass through 0, radius [m] cc > 0, radius [m] ccv < 0
	// wp1.param4 = 180.0;			// Yaw angle
	// wp1.x_lat = 47.3977422;
	// wp1.y_long = 8.5455943;
	// wp1.z_alt = 5.0;
	// missionMsg.request.waypoints.push_back(wp1);

	// mavros_msgs::Waypoint wp2;
	// wp2.frame = FRAME_GLOBAL_REL_ALT;
	// wp2.command = NAV_WAYPOINT;
	// wp2.is_current = false;
	// wp2.autocontinue = true;
	// wp2.param1 = 5.0;				// Hold time [decimal seconds]
	// wp2.param2 = 1.0;				// Acceptance radius [m]
	// wp2.param3 = 0.0;				// Pass through 0, radius [m] cc > 0, radius [m] ccv < 0
	// wp2.param4 = 180.0;			// Yaw angle
	// wp2.x_lat = 47.3977430;
	// wp2.y_long = 8.5455940;
	// wp2.z_alt = 4.0;
	// missionMsg.request.waypoints.push_back(wp2);

	// mavros_msgs::Waypoint wp3;
	// wp3.frame = FRAME_GLOBAL_REL_ALT;
	// wp3.command = NAV_LAND;
	// wp3.is_current = false;
	// wp3.autocontinue = true;
	// //wp3.param1 = 5.0;
	// //wp3.param2 = 1.0;
	// //wp3.param3 = 0.0;
	// wp3.param4 = 0.0;
	// wp3.x_lat = 47.3977422;
	// wp3.y_long = 8.5455930;
	// wp3.z_alt = 5.0;
	// missionMsg.request.waypoints.push_back(wp3);

	if (uploadMissionServiceClient.call(missionMsg) && missionMsg.response.success && missionMsg.response.wp_transfered == numberOfWaypoints )
	{
    	ROS_INFO("Uploded waypoints: %i",missionMsg.response.wp_transfered);
	}
	else
	{
    	ROS_ERROR("Mission upload failed. Uploded waypoints: %i",missionMsg.response.wp_transfered);
		res.result = ERROR_UPLOAD_MISSION;
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
			res.result = SUCCESS;
			ROS_INFO("Waypoints cleared. Succes: %i Result %i", msg.response.success);
		}
		else
		{
			res.result = ERROR;
			ROS_ERROR("Failed to clear mission. Succes: %i Result %i", msg.response.success);
		}
	}
	else
	{
		ROS_ERROR("Failed to clear mission. Succes: %i Result %i", msg.response.success);
		res.result = ERROR;
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
			res.result = SUCCESS;
		}
		else
		{
			ROS_ERROR("Failed to arm. Succes: %i Result %i", msg.response.success, msg.response.result);
			res.result = ERROR;
		}
	}
	else
	{
		ROS_ERROR("Failed to arm. Succes: %i Result %i", msg.response.success, msg.response.result);
		res.result = ERROR;		
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
			res.result = SUCCESS;
		}
		else
		{
			ROS_ERROR("Failed to disarm. Succes: %i Result %i", msg.response.success, msg.response.result);
			res.result = ERROR;
			
		}
	}
	else
	{
		ROS_ERROR("Failed to disarm. Succes: %i Result %i", msg.response.success, msg.response.result);
		res.result = ERROR;
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
			res.result = SUCCESS;
		}
		else
		{
			res.result = ERROR;
			ROS_ERROR("Failed to take off. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
	}
	else
	{
		res.result = ERROR;
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
			res.result = SUCCESS;
		}
		else
		{
			res.result = ERROR;
			ROS_ERROR("Failed to land. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
	}
	else
	{
		res.result = ERROR;
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
			res.result = SUCCESS;
		}
		else
		{
			res.result = ERROR;
			ROS_ERROR("Failed to start mission. Succes: %i Result %i", msg.response.success, msg.response.result);
		}
	}
	else
	{
		res.result = ERROR;
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
	ros::Rate rate(20);
	while(ros::ok())
	{
		dc.checkHeartbeat();
		ros::spinOnce();
		rate.sleep();
	}
  return 0;
}
