#include <ros/ros.h>
#include "geofence.h"
#include <vector>

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define PATH_PLANNER_CLASS path_planner


class PATH_PLANNER_CLASS
{
	// TODO Implement service planPath()
};

int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(PATH_PLANNER_CLASS));
  PATH_PLANNER_CLASS pp;

// GEOFENCE TESTING!
	geofence fence;
	//fence.set("home/mathias/Downloads/Geofence.kml","...");
	point p1, p2, p3, p4, ptest;
	p1.lon = 1.0;
	p1.lat = 1.0;
	p2.lon = 2.0;
	p2.lat = 1.0;
	p3.lon = 2.0;
	p3.lat = 2.0;
	p4.lon = 1.0;
	p4.lat = 2.0;
	std::vector<point> points;
	points.push_back(p1);
	points.push_back(p2);
	points.push_back(p3);
	points.push_back(p4);
	ptest.lon = 1.5;
	ptest.lat = 1.5;
	ptest.alt = 10;

	//bool result = fence.inside_polygon(points,ptest);
	obstacle o1;
	std::vector<point> opoints;
	point op;
	op.lon = 1.2;
	op.lat = 1.4;
	o1.points.push_back(op);
	op.lon = 1.2;
	op.lat = 1.6;
	o1.points.push_back(op);
	op.lon = 1.8;
	op.lat = 1.4;
	o1.points.push_back(op);
	op.lon = 1.8;
	op.lat = 1.6;
	o1.points.push_back(op);
	o1.height = 10;
	std::vector<obstacle> obstacles;
	obstacles.push_back(o1);
	fence.set_fence(points);
	fence.set_obstacles(obstacles);
	fence.set_max_altitude(30);


	point ptest2, ptest3;
	ptest2.lon = 1.5;
	ptest2.lat = 1.2;
	ptest2.alt = 11;
	ptest3.lon = 1.5;
	ptest3.lat = 1.8;
	ptest3.alt = 11;
	bool result = fence.edge_legal(ptest2,ptest3);

	//bool result = fence.point_legal(ptest);
	//ROS_INFO("%i",points.size());

	ROS_INFO("main result: %i",result);

	point geodetic_p;
	geodetic_p.lat = 55.0000000000;
	geodetic_p.lon = 009.0000000000;
	//geodetic_p.lat = 55.403756;
	//geodetic_p.lon = 10.40237;
	//ROS_INFO("%f,%f",geodetic_p.lat,geodetic_p.lon);
	//fence.geodetic_to_UTM(geodetic_p);
	//ROS_INFO("%f,%f",geodetic_p.lat,geodetic_p.lon);
	//fence.test_UTM();
	fence.self_test();
	ros::spin();
  return 0;
}
