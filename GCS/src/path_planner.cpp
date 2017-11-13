#include "path_planner.h"
#include <math.h>
#include <algorithm>
#include <iostream>

#define DEBUGGING true
#define SHRINK_FACTOR 0.90F
#define SHRINK_METERS 1.0F
#define MAX_WAYPOINT_DISTANCE 900

Path_planner::Path_planner(geofence &fence)
{
    this->fence = &fence;
}

void Path_planner::set_nodes(std::vector<point> points)
{
    for(unsigned int i = 0; i<points.size(); i++)
    {
        nodes.push_back(Node(points[i], i+1));
    }
}

// Missing: Should the altitude be included?
double Path_planner::heuristic_dist(Node *start, Node *goal)
{
    return sqrt(pow((start->coordinate.lat - goal->coordinate.lat),2) + pow((start->coordinate.lon - goal->coordinate.lon),2)
    + pow((start->coordinate.alt - goal->coordinate.alt),2));
}

Node* Path_planner::get_node_with_lowest_fscore(std::vector<Node*> &openset)
{
    Node* result;
    if(openset.size() > 0)
         result = openset[0];
    else
        return nullptr;

    for(unsigned int i = 0; i<openset.size(); i++)
    {
        if(openset[i]->fscore < result->fscore)
            result = openset[i];
    }
    return result;
}

void Path_planner::remove_node(std::vector<Node*> &openset, Node* node)
{
    for(unsigned int i = 0; i<openset.size(); i++)
    {
        if(openset[i] == node)
        {
            openset.erase(openset.begin()+i);
            return;
        }
    }
}

bool Path_planner::is_node_in_set(std::vector<Node*> set, Node* node)
{
    for(unsigned int i = 0; i<set.size(); i++)
    {
        if(node == set[i])
            return true;
    }
    return false;
}

// Connect all the nodes which are allowed
void Path_planner::connect_all_nodes()
{
    int k = 0;
    for(unsigned int i = 0; i<nodes.size(); i++)
    {
        for(unsigned int j = 0; j<nodes.size(); j++)
        {
            //std::cout << "connected: " << k++ << std::endl;
            if(i != j && fence->edge_legal(nodes[i].coordinate, nodes[j].coordinate))   //fence->edge_legal(nodes[i].coordinate, nodes[j].coordinate)
            {
                nodes[i].neighbors.push_back(&nodes[j]);
            }
        }
        //std::cout << "Neighbors: " << nodes[i].neighbors.size() << std::endl;
    }
}

// Creates a vector containing the path of nodes from start to goal
std::vector<Node*> Path_planner::reconstruct_path(Node *current)
{
    std::vector<Node*> final_path = {current};
    while(final_path.back()->parent != nullptr)
    {
        final_path.push_back(final_path.back()->parent);
    }
    return final_path;
}

// A* algorithm
std::vector<Node*> Path_planner::a_star(Node *start, Node *goal)
{
    std::vector<Node*> closedset;
    std::vector<Node*> openset = {start};

    start->gscore = 0;
    start->fscore = heuristic_dist(start, goal);

    while(openset.size() != 0)
    {
        Node *current = get_node_with_lowest_fscore(openset);
        if(current == goal)
            return reconstruct_path(current);

        remove_node(openset, current);
        closedset.push_back(current);

        //if(DEBUGGING) current->print_node();    // Debugging

        for(unsigned int i = 0; i<current->neighbors.size(); i++)
        {
            if(is_node_in_set(closedset, current->neighbors[i])) // If neighbor is in the closedset
                continue;   // Ignore the neighbor as it is already evaluated

            if(!is_node_in_set(openset, current->neighbors[i]))
                openset.push_back(current->neighbors[i]);   // Discover new node

            double tentative_gscore = current->gscore + heuristic_dist(current, current->neighbors[i]); // The distance from start and to current's neighbor
            if(tentative_gscore >= current->neighbors[i]->gscore)
                continue; // This is not a better path

            // This is the best path until now, so we saves it
            current->neighbors[i]->parent = current;
            current->neighbors[i]->gscore = tentative_gscore;
            current->neighbors[i]->fscore = tentative_gscore + heuristic_dist(current->neighbors[i], goal);
        }
    }
    if(DEBUGGING)  ROS_INFO("A* pathplanner failed in finding a path!");
    return std::vector<Node*>();    // Returns empty vector when pathplanning fails
    // Missing make sure this works
}

gcs::waypoint Path_planner::convert_node_to_waypoint(Node *node)
{
    gcs::waypoint point;
    point.lat = node->coordinate.lat;
    point.lon = node->coordinate.lon;
    point.alt = CRUISE_HEIGHT;//node->coordinate.alt;
    point.type = WAYPOINT;
        //goal.alt = ;
    return point;
}

gcs::path Path_planner::plan_path(point start, point goal)
{
	gcs::path waypoint_path;
	
	if(!fence->point_legal(start, LatLon) && !fence->point_legal(goal, LatLon))
	{
		if(DEBUGGING) ROS_INFO("Start or goal coordinates are not within the geofence!");
		return waypoint_path;	// Remember to look at this in the node
	}
	
    if(DEBUGGING) ROS_INFO("Shrinking the polygon...\n");
    shrink_polygon2(SHRINK_METERS);
    export_as_csv("/home/tobias/drone_ws/src/UASD_GCS/src/fences/testfile.txt");

    nodes.push_back(Node(start, -1));
    nodes.push_back(Node(goal, -2));

    //std::cout << "Is start point inside: " << fence->point_legal(start, UTM) << std::endl;

    if(DEBUGGING) ROS_INFO("Connecting all nodes...\n");
    connect_all_nodes();    // Connect all nodes with each other

    // Run a-star from start to goal
    if(DEBUGGING) ROS_INFO("A* pathplanning is started...\n"); 
    std::vector<Node*> path_nodes = a_star(&nodes[nodes.size() - 2], &nodes.back());
    if(DEBUGGING) ROS_INFO("A* pathplanning is finished.\n"); 

    // Interpolate between waypoints, if the distance between is to big
    //path_nodes = interpolate_path(path_nodes);

    // Convert nodes into a path of waypoints
    //gcs::path waypoint_path;
    for(unsigned int i = 0; i<path_nodes.size(); i++)
    {
        fence->UTM_to_geodetic(path_nodes[i]->coordinate);
        if(DEBUGGING) path_nodes[i]->print_node();
        waypoint_path.path.push_back(convert_node_to_waypoint(path_nodes[i]));
    }

    return waypoint_path;   // Missing: check if this works OBS fix so it returns lat and long instead of UTM
}

// Interpolate between waypoints, if the distance between is to big
std::vector<Node*> Path_planner::interpolate_path(std::vector<Node*> nodes)
{
    std::vector<Node*> new_path;
    for(unsigned int i = 0; i<nodes.size() - 1; i++)
    {
        // Create direction vector
        double vec_x = nodes[i+1]->coordinate.lat - nodes[i]->coordinate.lat;
        double vec_y = nodes[i+1]->coordinate.lon - nodes[i]->coordinate.lon;
        double vec_z = nodes[i+1]->coordinate.alt - nodes[i]->coordinate.alt;
        double length = std::sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z);

        if(length > MAX_WAYPOINT_DISTANCE)
        {
            double x_temp, y_temp, z_temp;
            int steps = length/MAX_WAYPOINT_DISTANCE;

            new_path.push_back(nodes[i]);   // Push start point

            // Interpolate
            int step = 0;
            while(step <= steps)
            {
                x_temp = nodes[i]->coordinate.lat + vec_x*( (double)step/(double)steps);
                y_temp = nodes[i]->coordinate.lon + vec_y*((double)step/(double)steps);
                z_temp = nodes[i]->coordinate.alt + vec_z*((double)step/(double)steps);
                Node temp_node;
                temp_node.coordinate.lat = x_temp;
                temp_node.coordinate.lon = y_temp;
                temp_node.coordinate.alt = z_temp;
                new_path.push_back(&temp_node);
                step++;
                std::cout << "Steps: " << steps << " step: " << step << std::endl;
            }
            new_path.push_back(nodes[i+1]); // Push end point
        }
    }
    return new_path;
}

void Path_planner::shrink_polygon(double shrink_factor)
{
    // Find polygon centroid
    point centroid;
    centroid.lat = 0;
    centroid.lon = 0;
    for(unsigned int i = 0; i<nodes.size(); i++)
    {
        centroid.lat += nodes[i].coordinate.lat;
        centroid.lon += nodes[i].coordinate.lon;
    }
    centroid.lat = centroid.lat / nodes.size();
    centroid.lon = centroid.lon / nodes.size();

    /*
    std::cout << "centroid: " << centroid.lat << " " << centroid.lon << std::endl;
    typedef std::numeric_limits< double > dbl;  // DEBUGGING
    std::cout.precision(dbl::max_digits10);
    std::cout << "DEBUGGING START: " << std::endl;*/
    // Translate to origin, scale/shrink, translate back
    for(unsigned int i = 0; i<nodes.size(); i++)
    {
        nodes[i].coordinate.lat = (nodes[i].coordinate.lat - centroid.lat)*shrink_factor + centroid.lat;
        nodes[i].coordinate.lon = (nodes[i].coordinate.lon - centroid.lon)*shrink_factor + centroid.lon;
        //if(DEBUGGING) nodes[i].print_node();
    }
}

void Path_planner::shrink_polygon2(double shrink_meters)
{
    std::vector<Node> temp_nodes;

    typedef std::numeric_limits< double > dbl;
    if(DEBUGGING) std::cout.precision(dbl::max_digits10);

    for(unsigned int i = 0; i<nodes.size(); i++)
    {
        Node lpoint;
        Node mpoint;
        Node rpoint;
        int index;

        // Left point
        if(i == 0) index = nodes.size() - 1;
        else index = i - 1;
        lpoint = nodes[index];

        // Middle point
        mpoint = nodes[i];

        // Right point
        if(i == nodes.size()-1) index = 0;
        else index = i + 1;
        rpoint = nodes[index];

        // Calculate two vector from the middle point to the other points
        Vect vec1;
        Vect vec2;
        vec1.x = lpoint.coordinate.lat - mpoint.coordinate.lat;
        vec1.y = lpoint.coordinate.lon - mpoint.coordinate.lon;
        vec2.x = rpoint.coordinate.lat - mpoint.coordinate.lat;
        vec2.y = rpoint.coordinate.lon - mpoint.coordinate.lon;

        // Calculate a unit-vector beweeen the two other vectors
        Vect vec3 = vec1 + vec2;
        vec3 = vec3.unit_vect();

        // Make a copy of the middle node and move it a distance in the direction of the unit-vector
        Node new_node = nodes[i];

        /*
        if(DEBUGGING) std::cout << "Unit-vector x: " << vec3.x << " y: " << vec3.y << std::endl;
        if(DEBUGGING) new_node.print_node();
        if(DEBUGGING)
        {
            Node temp = new_node;
            fence->UTM_to_geodetic(temp.coordinate);
            temp.print_node();
        }*/

        point new_point1;
        point new_point2;
        new_point1.lat = new_node.coordinate.lat + vec3.x * shrink_meters;
        new_point1.lon = new_node.coordinate.lon + vec3.y * shrink_meters;
        new_point2.lat = new_node.coordinate.lat + vec3.x * -shrink_meters;
        new_point2.lon = new_node.coordinate.lon + vec3.y * -shrink_meters;

        if(fence->point_legal(new_point1, UTM))
            new_node.coordinate = new_point1;
        else if(fence->point_legal(new_point2, UTM))
            new_node.coordinate = new_point2;
        else
            ROS_INFO("Path_planner: An error occured shrinking the polygon!");

        /*
        if(DEBUGGING) new_node.print_node();
        if(DEBUGGING)
        {
            Node temp = new_node;
            fence->UTM_to_geodetic(temp.coordinate);
            temp.print_node();
        }
        std::cout << "\n";*/

        temp_nodes.push_back(new_node);
    }
    nodes = temp_nodes;
}

void Path_planner::export_as_csv(std::string filename)
{
    std::ofstream csvfile(filename);
    typedef std::numeric_limits< double > dbl;
    csvfile.precision(dbl::max_digits10);   // Set max precision outputs to file

    if (csvfile.is_open())
    {
        csvfile << "Name,Description,Latitude,Longitude\n";
        for(unsigned int i = 0; i<nodes.size(); i++)
        {
            point temp_point = nodes[i].coordinate;
            fence->UTM_to_geodetic(temp_point);
            csvfile << i+1 << i+1 << ",none," << temp_point.lat << "," << temp_point.lon << "\n";
        }

        csvfile.close();
    }
}

void Path_planner::print_nodes()
{
    for(unsigned int i = 0; i<nodes.size(); i++)
    {
        nodes[i].print_node();
    }
}
