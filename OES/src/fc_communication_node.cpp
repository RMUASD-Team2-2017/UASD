#include <ros/ros.h>

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define NODE_CLASS_NAME fc_comm

class NODE_CLASS_NAME
{

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(NODE_CLASS_NAME));
  NODE_CLASS_NAME oes;
  ros::spin();
  return 0;
}
