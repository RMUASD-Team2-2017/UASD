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
#define CRUISE_HEIGHT 30.0

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
    if(isPlanning)
	{
		res.result = 1;
	}
	else
	{
		start = req.origin;
		goal = req.goal;
		isPlanning = true;
		res.result = 0;
	}
}

gcs::path planPath()
{
    gcs::path planned_path;
    geofence fence;
    Path_planner planner(fence, CRUISE_HEIGHT, MAX_WAYPOINT_DISTANCE, SHRINK_METERS);
    fence.set_max_altitude(200.0);

    ROS_INFO("Path planning started");

    // Find the path to the geofence file
    std::string geofence_file;
    ros::param::param<std::string>("/geofence_file", geofence_file, ros::package::getPath("gcs") + "/src/fences/model_airfield_fence.csv");

    // Load geofence file
    bool fence_load_sucess= fence.set_fence_csv(geofence_file, true);

    if(fence_load_sucess)
    {
        // Set geofence points as nodes in the pathplanner
        planner.set_nodes(fence.get_fence());

        point start_point, goal_point;
        start_point.lat = start.lat;    //modelfield: 55.471935; BogenseMiddle: 55.555682; BogenseLong: 55.554359; //start.lat;
        start_point.lon = start.lon;    //modelfield: 10.414690; BogenseMiddle: 10.113780; BogenseLong: 10.113565; //start.lon;
        start_point.alt = 0.0;
        goal_point.lat = goal.lat;  // modelfield: 55.472072; BogenseLong: 55.567094; Bogenseshort: 55.558805 //goal.lat;
        goal_point.lon = goal.lon;  // modelfield: 10.416813; BogenseLong: 10.121619; Bogenseshort: 10.113543 //goal.lon;
        goal_point.alt = 0.0;

        //std::cout << start_point.lat << " " << start_point.lon << std::endl;
        //std::cout << goal_point.lat << " " << goal_point.lon << std::endl;

        fence.geodetic_to_UTM(start_point);
        fence.geodetic_to_UTM(goal_point);

        // Plan path
        planned_path = planner.plan_path(start_point,goal_point);

        // Make TakeOff, WayPoint and Land types for mavlink
        if(planned_path.path.size())
        {
            planned_path.path[0].alt = CRUISE_HEIGHT;   // Make the first waypoint a takeoff type
            planned_path.path[0].type = TAKEOFF;

            // Add an extra waypoint
            gcs::waypoint point = planned_path.path.back();
            planned_path.path.back().alt = CRUISE_HEIGHT;
        	planned_path.path.back().type = WAYPOINT;
            planned_path.path.push_back(point);

            planned_path.path.back().alt = 0;
        	planned_path.path.back().type = LAND;  // Make the last waypoint a landing type
        }

        ROS_INFO("Pathplanning succeded!");
    }
    return planned_path;
}


int main(int argc, char** argv)
{
	ros::init(argc, argv, VALUE_AS_STRING(PATH_PLANNER_CLASS));
	ros::NodeHandle n;
    ros::ServiceServer planPathService = n.advertiseService("/path_planner/plan",callbackPlanPathService);
    ros::Publisher pathPublisher = n.advertise<gcs::path>("/path_planner/path",1);

    // Only for testing
    //gcs::path plannedPath = planPath();

	ros::Rate rate(20);

	while(ros::ok())
	{
        if(isPlanning)
        {
            gcs::path plannedPath = planPath();
            if(plannedPath.path.size())
                pathPublisher.publish(plannedPath);
            else
                ROS_ERROR("Pathplanning failed!");
            isPlanning = false;
        }

		ros::spinOnce();
		rate.sleep();
	}
    return 0;
}
