#include <ros/ros.h>
// TODO Include custom message type dock_data_msg
// TODO Include GPS message type
// TODO Include mavlink to/from message types
// TODO Include waypoint list message type

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define NODE_CLASS_NAME oes_control

class NODE_CLASS_NAME
{
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

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(NODE_CLASS_NAME));
  NODE_CLASS_NAME oes;
  ros::spin();
  return 0;
}
