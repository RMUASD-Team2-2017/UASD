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
#include <ros/package.h>

// Defines
#define MAX_WAYPOINT_DISTANCE 900.0
#define SHRINK_METERS 1.0F
#define CRUISE_HEIGHT 5.0

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

    if(plannedPath.path.size())
    {
        pathPublisher.publish(plannedPath);
        res.result = 1;
        ROS_INFO("Pathplanning succeded!");
    }
    else
    {
        res.result = 0;
        ROS_ERROR("Pathplanning failed!");
    }
}

gcs::path planPath()
{
    gcs::path planned_path;
    geofence fence;
    Path_planner planner(fence, CRUISE_HEIGHT, MAX_WAYPOINT_DISTANCE, SHRINK_METERS);
    fence.set_max_altitude(200.0);

    // Find the path to the geofence file
    std::string geofence_file;
    ros::param::param<std::string>("/geofence_file", geofence_file, ros::package::getPath("gcs") + "/src/fences/geofence.csv");

    // Load geofence file
    bool fence_load_sucess= fence.set_fence_csv(geofence_file, true);

    if(fence_load_sucess)
    {
        // Set geofence points as nodes in the pathplanner
        planner.set_nodes(fence.get_fence());

        point start_point, goal_point;
        start_point.lat = /*55.554359;//*/start.lat;
        start_point.lon = /*10.113565;//*/start.lon;
        start_point.alt = 0.0;
        goal_point.lat =  /*55.567094;//*/goal.lat;
        goal_point.lon =  /*10.121619;//*/goal.lon;
        goal_point.alt = 0.0;

        //std::cout << start_point.lat << " " << start_point.lon << std::endl;
        //std::cout << goal_point.lat << " " << goal_point.lon << std::endl;

        fence.geodetic_to_UTM(start_point);
        fence.geodetic_to_UTM(goal_point);

        // Plan path
        planned_path = planner.plan_path(start_point,goal_point);
        if(planned_path.path.size())
        {
            planned_path.path[0].alt = CRUISE_HEIGHT;   // Make the first waypoint a takeoff type
            planned_path.path[0].type = TAKEOFF;

            planned_path.path.back().alt = 0;
        	planned_path.path.back().type = LAND;  // Make the last waypoint a landing type
        }
    }

    return planned_path;
}



int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(PATH_PLANNER_CLASS));
	ros::NodeHandle n;
    ros::ServiceServer planPathService = n.advertiseService("/path_planner/plan",callbackPlanPathService);

    // Only for testing
    //gcs::path plannedPath = planPath();

	ros::Rate rate(20);

	while(ros::ok())
	{
		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
