#include <ros/ros.h>

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define GCS_CONTROL_CLASS gcs_control


class GCS_CONTROL_CLASS
{

};

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(GCS_CONTROL_CLASS));
  GCS_CONTROL_CLASS gcs;
  ros::spin();
  return 0;
}
