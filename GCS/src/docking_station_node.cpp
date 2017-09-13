#include <ros/ros.h>

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define DOCKING_STATION_CLASS docking_station


class DOCKING_STATION_CLASS
{

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(DOCKING_STATION_CLASS));
  DOCKING_STATION_CLASS ds;
  ros::spin();
  return 0;
}
