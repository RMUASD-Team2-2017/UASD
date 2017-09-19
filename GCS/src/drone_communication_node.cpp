#include <ros/ros.h>
// TODO Include mavlink to/from message types

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define DRONE_COMM_CLASS drone_comm


class DRONE_COMM_CLASS
{
public:
	DRONE_COMM_CLASS(ros::NodeHandle);
	~DRONE_COMM_CLASS();

private:
	ros::NodeHandle nh;

	// TODO Implement service uploadWaypoints()

	// TODO Implement service monitorFlight()

};

DRONE_COMM_CLASS::DRONE_COMM_CLASS(ros::NodeHandle n)
{
	nh = n;
}

DRONE_COMM_CLASS::~DRONE_COMM_CLASS()
{}


int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(DRONE_COMM_CLASS));
	ros::NodeHandle nh;
	DRONE_COMM_CLASS dc(nh);
  ros::spin();
  return 0;
}
