#include <ros/ros.h>
// TODO Include custom message type dock_data_msg
// TODO Include GPS message type
// TODO Include mavlink to/from message types
// TODO Include waypoint list message type

#include "gcs/planPath.h"
#include "gcs/path.h"
#include "gcs/waypoint.h"
#include "gcs/uploadMission.h"
#include "gcs/arm.h"


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

		ros::Subscriber path_subscriber;
		void pathSubscriberCallback(const gcs::path::ConstPtr& msg);
		

		/* Methods */ 


		/* Attributes */
		gcsState state = IDLE;
		missionUploadState mission_upload_state = MISSION_IDLE;
		gcs::path planned_path;

	// 1 Monitor docking station
	// - Call service monitorDock() from docking_station_node
	// TODO

	// 2 Receive distress call from database
	// - Http get GPS position and timestamp from database
	// TODO
	
	// 3 Initiate pre-flight check
	// - Call service preflight() from pre_flight_node
	// TODO
	
	// 4 Perform path planning
	// - Call service planPath() from path_planner_node
	// TODO
	
	// 5 Open docking station
	// - Call service openDock() from docking_station_node
	// TODO
	
	// 6 Upload path to drone and initiate flight
	// - Call service uploadWaypoints() from drone_communication_node
	// TODO
	
	// 7 Monitor the flight (Error procedure)
	// - Call service monitorFlight() from drone_communication_node subscribing to mavlink/from
	// - In case of extended link loss, do something
	// TODO
	
	// 8 Update database
	// - Http post flight information (heartbeat, battery voltage, etc.) to database
	// TODO

};

GCS_CONTROL_CLASS::GCS_CONTROL_CLASS(ros::NodeHandle n)
{
	nh = n;

	plan_path_service_client = nh.serviceClient<gcs::planPath>("/path_planner/plan");
	upload_mission_service_client = nh.serviceClient<gcs::uploadMission>("/drone_communication/uploadMission");
	arm_service_client = nh.serviceClient<gcs::arm>("/drone_communication/arm");
	
	path_subscriber = nh.subscribe<gcs::path>("/path_planer/path",1,&GCS_CONTROL_CLASS::pathSubscriberCallback,this);

}

void GCS_CONTROL_CLASS::run()
{
	switch (state)
	{
		case IDLE:
			{
				// Continuosly monitor drone and docking station
			}
			break;
		case RECEIVED_DISTRESS_CALL:
			{
				// Start preflight check
				
				// Start path planning
				gcs::planPath path_request;
				path_request.request.origin.lat = 55.0;
				path_request.request.origin.lon = 10.0;
				path_request.request.origin.alt = 10.0;
				path_request.request.goal.lat = 55.5;
				path_request.request.goal.lon = 10.0;
				path_request.request.goal.alt = 10.0;
				if ( !plan_path_service_client.call(path_request) )
					ROS_ERROR("Plan path failed");
				state = PREPARE;
			}
			break;
		case PREPARE:
			{
				if( mission_upload_state == PATH_RECEIVED )
				{
					gcs::uploadMission mission_upload_msg;
					mission_upload_msg.request.waypoints = planned_path;
					if( upload_mission_service_client.call(mission_upload_msg) && mission_upload_msg.response.result == SUCCESS )
					{
						mission_upload_state = UPLOAD_SUCCESS;
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
					state = WAIT_FOR_READY;
				}
			}
			break;
		case WAIT_FOR_READY:
			{
				// wait for opening of docking station
				state = DEPLOY;
			}
			break;
		case DEPLOY:
			{
				//If everything ok -> arm the drone
				gcs::arm arm_msg;
				if( arm_service_client.call(arm_msg) && arm_msg.response.result == SUCCESS)
				{
					state = FLYING;
				}
				else
					ROS_ERROR("Failed to arm");
			}
			break;
		case FLYING:
			{
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
