#include <ros/ros.h>

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define PATH_PLANNER_CLASS path_planner


class PATH_PLANNER_CLASS
{

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(PATH_PLANNER_CLASS));
  PATH_PLANNER_CLASS pp;
  ros::spin();
  return 0;
}
