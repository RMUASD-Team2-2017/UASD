#include <ros/ros.h>
// TODO Include custom message type dock_data_msg

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define DOCKING_STATION_CLASS docking_station


class DOCKING_STATION_CLASS
{
	// TODO Implement topic dock_data

	// TODO Implement service monitorDock()
	//				- Read temperature sensor
	//				- Read humidity sensor
	//				- Publish sensor data on dock_data
	
	// TODO Implement service openDock()

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(DOCKING_STATION_CLASS));
  DOCKING_STATION_CLASS ds;
  ros::spin();
  return 0;
}
