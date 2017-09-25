#include <ros/ros.h>
// TODO Include custom message type dock_data_msg
// TODO Include GPS message type
// TODO Include mavlink to/from message types
// TODO Include waypoint list message type

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define GCS_CONTROL_CLASS gcs_control

enum gcsState {
	idle,
	received_distress_call,
	prepare,
	wait_for_ready,
	deploy,
	flying,
	arrived
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

		/* Methods */ 


		/* Attributes */
		gcsState state = idle;

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
}

	// idle,
	// received_distress_call,
	// wait_for_ready,
	// deploy,
	// flying,
	// arrived

void GCS_CONTROL_CLASS::run()
{
	switch (state)
	{
		case idle:
			// Continuosly monitor drone and docking station
			break;
		case received_distress_call:
			// Start preflight check
			// Start path planning
		case prepare:
			// upload path (mission) when ready
			// if weather ok, open docking station
			break;
		case wait_for_ready:
			// wait for opening of docking station
			break;
		case deploy:
			//If everything ok -> arm the drone
			break;
		case flying:
			// Monitor during flight
			break;
		case arrived:
			// ?
			break;
		default:
			ROS_ERROR("Bad state");
			/* Default Code */
	}
	
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
	}
	return 0;
}
