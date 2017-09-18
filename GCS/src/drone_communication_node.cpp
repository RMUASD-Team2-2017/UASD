#include <ros/ros.h>
// TODO Include mavlink to/from message types

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define DRONE_COMM_CLASS drone_comm


class DRONE_COMM_CLASS
{
	// TODO Implement service uploadWaypoints()
	
	// TODO Implement service monitorFlight()

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(DRONE_COMM_CLASS));
  DRONE_COMM_CLASS dc;
  ros::spin();
  return 0;
}
