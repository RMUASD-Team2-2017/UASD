#include <ros/ros.h>

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define GCS_CONTROL_CLASS gcs_control


class GCS_CONTROL_CLASS
{
	// Monitor docking station

	// Receive distress call from database
	
	// Initiate pre-flight check
	
	// Open docking station
	
	// Perform path planning
	
	// Upload path to drone and initiate flight
	
	// Monitor the flight
	
	// Send data to Flight Control Center

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(GCS_CONTROL_CLASS));
  GCS_CONTROL_CLASS gcs;
  ros::spin();
  return 0;
}
