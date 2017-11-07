#include <ros/ros.h>

#include "gcs/planPath.h"
#include "gcs/path.h"
#include "gcs/waypoint.h"
#include "gcs/uploadMission.h"
#include "gcs/arm.h"
#include "gcs/deploy_request.h"
#include "gcs/startMission.h"
#include "gcs/dockData.h"
#include "gcs/openDock.h"
#include "gcs/preFlight.h"
#include "gcs/communicationStatus.h"


#include "sensor_msgs/NavSatFix.h"

#include "mavros_msgs/WaypointList.h"

#include "std_msgs/String.h"
#include "std_msgs/Bool.h"


#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define GCS_CONTROL_CLASS gcs_control

#define SUCCESS					0
#define ERROR_CLEARING_MISSION 	1
#define ERROR_PARAM_SET 		2
#define ERROR_SET_MODE			3

enum gcsState {
	IDLE,
	RECEIVED_DISTRESS_CALL,
	PREPARE,
	WAIT_FOR_READY,
	DEPLOY,
	FLYING,
	ARRIVED
};

enum missionUploadState {
	MISSION_IDLE,
	PATH_RECEIVED,
	UPLOAD_SUCCESS,
	UPLOAD_ERROR
};

class GCS_CONTROL_CLASS
{
	public:
		GCS_CONTROL_CLASS(ros::NodeHandle);
		~GCS_CONTROL_CLASS();
		void run();
	private:
		/* ROS */
		ros::NodeHandle nh;

		ros::ServiceClient plan_path_service_client;
		ros::ServiceClient upload_mission_service_client;
		ros::ServiceClient arm_service_client;
		ros::ServiceClient start_mission_service_client;
		ros::ServiceClient open_dock_service_client;
		ros::ServiceClient pre_flight_service_client;

		ros::Subscriber path_subscriber;
		void pathSubscriberCallback(const gcs::path::ConstPtr& msg);
		ros::Subscriber deploy_subscriber;
		void deploySubscriberCallback(const gcs::deploy_request::ConstPtr& msg);
		ros::Subscriber drone_gps_subscriber;
		void droneGpsSubscriberCallback(const sensor_msgs::NavSatFix::ConstPtr& msg);
		ros::Subscriber mission_state_subscriber;
		void missionStateSubscriberCallback(const mavros_msgs::WaypointList::ConstPtr& msg);
		ros::Subscriber dock_data_subscriber;
		void dockDataSubscriberCallback(const gcs::dockData::ConstPtr& msg);

		ros::Publisher uav_state_publisher;
		ros::Publisher mission_completed_publisher;
		ros::Publisher communicationStatusPublisher;

		/* Methods */


		/* Attributes */
		gcsState state = IDLE;
		missionUploadState mission_upload_state = MISSION_IDLE;
		gcs::path planned_path;
		gcs::deploy_request deploy_request;
		sensor_msgs::NavSatFix current_drone_position;
		int current_waypoint = 0;
		int last_waypoint = 0;
		int number_of_waypoints = 0;
		// gcs::dockData dock_sensor_data;
		float dock_temperature = 0, dock_humidity = 0;
		std::vector<double> dock_voltage = {0, 0, 0, 0};
		double outside_temperature = 0, outside_humidity = 0, wind_speed = 0;
		double battery_voltage = 0;

	// 1 Monitor docking station
	// - Call service monitorDock() from docking_station_node
	// Implemented as dockData topic

	// 2 Receive distress call from database
	// - Http get GPS position and timestamp from database

	// 3 Initiate pre-flight check
	// - Call service preflight() from pre_flight_node

	// 4 Perform path planning
	// - Call service planPath() from path_planner_node

	// 5 Open docking station
	// - Call service openDock() from docking_station_node

	// 6 Upload path to drone and initiate flight
	// - Call service uploadWaypoints() from drone_communication_node

	// 7 Monitor the flight (Error procedure)
	// - Call service monitorFlight() from drone_communication_node subscribing to mavlink/from
	// - In case of extended link loss, do something

	// 8 Update database
	// - Http post flight information (heartbeat, battery voltage, etc.) to database

};

GCS_CONTROL_CLASS::GCS_CONTROL_CLASS(ros::NodeHandle n)
{
	nh = n;

	plan_path_service_client = nh.serviceClient<gcs::planPath>("/path_planner/plan");
	upload_mission_service_client = nh.serviceClient<gcs::uploadMission>("/drone_communication/uploadMission");
	arm_service_client = nh.serviceClient<gcs::arm>("/drone_communication/arm");
	start_mission_service_client = nh.serviceClient<gcs::startMission>("drone_communication/startMission");
	open_dock_service_client = nh.serviceClient<gcs::openDock>("/docking_station/openDock");
	pre_flight_service_client = nh.serviceClient<gcs::preFlight>("/pre_flight_node/preFlight");

	path_subscriber = nh.subscribe<gcs::path>("/path_planner/path",1,&GCS_CONTROL_CLASS::pathSubscriberCallback,this);
	deploy_subscriber = nh.subscribe<gcs::deploy_request>("/web_interface/deploy_request",1,&GCS_CONTROL_CLASS::deploySubscriberCallback,this);
	drone_gps_subscriber = nh.subscribe<sensor_msgs::NavSatFix>("/drone_communication/globalPosition",1,&GCS_CONTROL_CLASS::droneGpsSubscriberCallback,this);
	mission_state_subscriber = nh.subscribe<mavros_msgs::WaypointList>("/drone_communication/missionState",1,&GCS_CONTROL_CLASS::missionStateSubscriberCallback, this);
	dock_data_subscriber = nh.subscribe<gcs::dockData>("/docking_station/dock_data",1,&GCS_CONTROL_CLASS::dockDataSubscriberCallback, this);

	uav_state_publisher = nh.advertise<std_msgs::String>("/web_interface/listen/set_uav_state",1);
	mission_completed_publisher = nh.advertise<std_msgs::Bool>("web_interface/listen/mission_done",1);
	communicationStatusPublisher = nh.advertise<gcs::communicationStatus>("/drone_communication/communiationStatus",1);

	std_msgs::String msg;
	msg.data = "idle";
	uav_state_publisher.publish(msg);
}

void GCS_CONTROL_CLASS::run()
{
	switch (state)
	{
		case IDLE:
			{
				if ( dock_temperature > 35 ||
					 dock_temperature< 15 )
				{
					ROS_INFO("Check docking station conditions.");
				}
				if ( dock_temperature > 50 ||
					 dock_temperature < 0 )
				{
					ROS_ERROR("Docking station conditions critical.");
				}
				// ROS_INFO("Battery cell voltages:\t%f\t%f\t%f\t%f",
				// 			dock_voltage[0], dock_voltage[1],
				// 			dock_voltage[2], dock_voltage[3]);

				// Continuosly monitor drone and docking station
				ROS_INFO("IDLE");
			}
			break;
		case RECEIVED_DISTRESS_CALL:
			{
				ROS_INFO("RECEIVED_DISTRESS_CALL");

				// Start path planning
				gcs::planPath path_request;
				path_request.request.origin.lat = current_drone_position.latitude;
				path_request.request.origin.lon = current_drone_position.longitude;
				path_request.request.origin.alt = current_drone_position.altitude;	// This doesn't matter. We use relative altitude frame in mavlink
				path_request.request.goal.lat = deploy_request.point.lat;
				path_request.request.goal.lon = deploy_request.point.lon;
				path_request.request.goal.alt = 0;
				if ( !plan_path_service_client.call(path_request) ) {
					ROS_ERROR("Plan path failed");
					// state unchanged
				}
				else state = PREPARE;
			}
			break;
		case PREPARE:
			{
				ROS_INFO("PREPARE");

				if( mission_upload_state == PATH_RECEIVED )
				{
					ROS_INFO("Trying to upload");
					gcs::uploadMission mission_upload_msg;
					mission_upload_msg.request.waypoints = planned_path;
					if( upload_mission_service_client.call(mission_upload_msg) && mission_upload_msg.response.result == SUCCESS )
					{
						mission_upload_state = UPLOAD_SUCCESS;
						ROS_INFO("Upload succeeded.");
					}
					else
					{
						mission_upload_state = UPLOAD_ERROR;
						ROS_ERROR("Failed to upload mission");
					}
				}
				// if weather ok, open docking station
				if(mission_upload_state == UPLOAD_SUCCESS ) // and weather ok
				{
					gcs::preFlight pre_flight_msg;
					bool pfcheck = pre_flight_service_client.call(pre_flight_msg);
					outside_temperature = pre_flight_msg.response.temperature;
					outside_humidity = pre_flight_msg.response.humidity;
					wind_speed = pre_flight_msg.response.windSpeed;
					battery_voltage = pre_flight_msg.response.voltage;
					// ROS_INFO("Pre-flight info: Tmp: %f\tHmd: %f\tSpd: %f\tVlt: %f\tLat: %f\tLon: %f",
					// 			outside_temperature, outside_humidity, wind_speed, battery_voltage,
					// 			pre_flight_msg.response.latitude, pre_flight_msg.response.longitude);
					if ( pfcheck && pre_flight_msg.response.result == true)
					{
						state = WAIT_FOR_READY;
						ROS_INFO("WAIT_FOR_READY");
					}
					else
					{
						ROS_ERROR("Pre-flight check failed.");
					}
				}
			}
			break;
		case WAIT_FOR_READY:
			{
				// wait for opening of docking station
				gcs::openDock open_dock_msg;
				if ( open_dock_service_client.call(open_dock_msg) && open_dock_msg.response.dockIsOpen == true)
				{
					state = DEPLOY;
					ROS_INFO("DEPLOY");
				}
				else
					ROS_ERROR("Failed to open docking station.");
			}
			break;
		case DEPLOY:
			{
				// TODO: See why we go here after killing the docking station node.
				ROS_INFO("DEPLOY");
				//If everything ok -> arm the drone
				gcs::startMission start_msg;
				if( start_mission_service_client.call(start_msg) && start_msg.response.result == SUCCESS)
				{
					state = FLYING;
					std_msgs::String msg;
					msg.data = "transport";
					uav_state_publisher.publish(msg);
					ROS_INFO("FLYING");

				}
				else
					ROS_ERROR("Failed to arm");

					// gcs::arm arm_msg;
					// if( arm_service_client.call(arm_msg) && arm_msg.response.result == SUCCESS)
					// {
					// 	state = FLYING;
					// 	std_msgs::String msg;
					// 	msg.data = "transport";
					// 	uav_state_publisher.publish(msg);
					// 	ROS_INFO("FLYING");
					//
					// }
					// else
					// 	ROS_ERROR("Failed to arm");
			}
			break;
		case FLYING:
			{
				if( last_waypoint == (number_of_waypoints-1) && current_waypoint == 0 )
				{
					state = ARRIVED;
					ROS_INFO("ARRIVED");
					std_msgs::String msg;
					msg.data = "landed";
					uav_state_publisher.publish(msg);
					std_msgs::Bool complete_msg;
					mission_completed_publisher.publish(complete_msg);
				}
				// Monitor during flight
			}
			break;
		case ARRIVED:
			{

				// ?
			}
			break;
		default:
			ROS_ERROR("Bad state");
			/* Default Code */
	}

}

void GCS_CONTROL_CLASS::pathSubscriberCallback(const gcs::path::ConstPtr& msg)
{
	planned_path = *msg;
	mission_upload_state = PATH_RECEIVED;
	ROS_INFO("Path callback");
}

void GCS_CONTROL_CLASS::deploySubscriberCallback(const gcs::deploy_request::ConstPtr& msg)
{
	deploy_request = *msg;
	state = RECEIVED_DISTRESS_CALL;
}

void GCS_CONTROL_CLASS::droneGpsSubscriberCallback(const sensor_msgs::NavSatFix::ConstPtr& msg)
{
	current_drone_position = *msg;
}

void GCS_CONTROL_CLASS::missionStateSubscriberCallback(const mavros_msgs::WaypointList::ConstPtr& msg)
{
	number_of_waypoints = msg->waypoints.size();
	last_waypoint = current_waypoint;
	for(int i = 0; i < msg->waypoints.size(); i++)
	{
		if( msg->waypoints[i].is_current == true)
		{
			current_waypoint = i;
			ROS_INFO("Reached waypoint %i", current_waypoint);
			break;
		}
	}
}

void GCS_CONTROL_CLASS::dockDataSubscriberCallback(const gcs::dockData::ConstPtr& msg)
{
	gcs::dockData dock_sensor_data = *msg;
	dock_temperature = dock_sensor_data.temperature;
	dock_humidity = dock_sensor_data.humidity;
	dock_voltage = dock_sensor_data.voltage;
	// dock_sensor_data = *msg;
}

GCS_CONTROL_CLASS::~GCS_CONTROL_CLASS()
{}

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(GCS_CONTROL_CLASS));
	ros::NodeHandle nh;
	GCS_CONTROL_CLASS gcs(nh);
	ros::Rate rate(20);
	while(ros::ok())
	{
		gcs.run();
		ros::spinOnce();
		rate.sleep();
	}
	return 0;
}
