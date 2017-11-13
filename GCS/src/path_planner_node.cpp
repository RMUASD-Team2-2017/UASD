//General
#include <vector>
#include <string>
#include <fstream>
#include <iostream>
#include <stdlib.h>

// Custom
#include "geofence.h"
#include "path_planner.h"

// ROS
#include <ros/ros.h>
#include "gcs/planPath.h"
#include "gcs/waypoint.h"
#include "gcs/path.h"

#define NAME_AS_STRING(macro) #macro
#define VALUE_AS_STRING(macro) NAME_AS_STRING(macro)

#define PATH_PLANNER_CLASS path_planner

bool isPlanning = false;
bool planning_succeded = true;
gcs::waypoint start;
gcs::waypoint goal;

gcs::path planPath();

bool callbackPlanPathService(gcs::planPath::Request &req, gcs::planPath::Response &res)
{
    ros::NodeHandle n;
    ros::Publisher pathPublisher = n.advertise<gcs::path>("/path_planner/path",1);
    start = req.origin;
    goal = req.goal;

    ROS_INFO("Path planning started");
    gcs::path plannedPath = planPath();
    ROS_INFO("Path planning has finished");

    if(plannedPath.path.size())
    {
        pathPublisher.publish(plannedPath);
        res.result = 1;
        ROS_INFO("Pathplanning succeded!");
    }
    else
    {
        res.result = 0;
        ROS_INFO("Pathplanning failed!");
    }


    /*
    if(isPlanning)
	{
		res.result = planning_succeded;
	}
	else
	{
		start = req.origin;
		goal = req.goal;
		isPlanning = true;
		res.result = 0;
	}*/
}

/*
void planPath()
{
	// NOTE: Implement a better path planner and use geofence
	origin_temp.alt = CRUISE_HEIGHT;
	origin_temp.type = TAKEOFF;
	plannedPath.path.push_back(origin_temp);
	gcs::waypoint goal_temp = goal;
	goal.alt = CRUISE_HEIGHT;
	goal.type = WAYPOINT;
	plannedPath.path.push_back(goal);
	gcs::waypoint goal_land = goal;
	goal_land.alt = 0;
	goal_land.type = LAND;
	plannedPath.path.push_back(goal_land);
	pathPublisher.publish(plannedPath);
	isPlanning = false;
}*/

gcs::path planPath()
{
    geofence fence;
    Path_planner planner(fence);
    fence.set_max_altitude(200.0);
    //fence.set_fence_csv("/home/tobias/drone_ws/src/UASD_GCS/src/fences/testfence.csv", false); // Testfence
    fence.set_fence_csv("/home/tobias/Dropbox/RobTek/Cand_3_semester/Unmanned Aerial System Design/UASD/GCS/src/fences/geofence.csv", true);
    planner.set_nodes(fence.get_fence());

    point start_point, goal_point;
    start_point.lat = 55.561142;//55.554453;//start.lat;
    start_point.lon = 10.113039;//10.112788;//start.lon;
    start_point.alt = 0.0;
    goal_point.lat =   55.565114;//goal.lat;
    goal_point.lon =  10.121754;///goal.lon;
    goal_point.alt = 0.0;

    std::cout << start_point.lat << " " << start_point.lon << std::endl;
    std::cout << goal_point.lat << " " << goal_point.lon << std::endl;

    fence.geodetic_to_UTM(start_point);
    fence.geodetic_to_UTM(goal_point);

    // Plan path
    gcs::path planned_path = planner.plan_path(start_point,goal_point);

    if(planned_path.path.size())
    {
        planned_path.path[0].alt = CRUISE_HEIGHT;
        planned_path.path[0].type = TAKEOFF;

        planned_path.path.back().alt = 0;
    	planned_path.path.back().type = LAND;
    }

    //isPlanning = false;
    return planned_path;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(PATH_PLANNER_CLASS));
	ros::NodeHandle n;
    ros::ServiceServer planPathService = n.advertiseService("/path_planner/plan",callbackPlanPathService);

    // Only for testing
    gcs::path plannedPath = planPath();

	ros::Rate rate(20);

	while(ros::ok())
	{
        /*
		if(isPlanning)
        {
            std::cout << "Path planning started!" << std::endl;
            gcs::path plannedPath = planPath();
            std::cout << "Path planning completed!" << std::endl;
            if(planned_path.path.size())
                pathPublisher.publish(plannedPath);

        }*/

		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
