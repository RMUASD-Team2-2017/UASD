#include "path_planner.h"
#include <math.h>
#include <algorithm>
#include <iostream>

#define DEBUGGING true
#define SHRINK_FACTOR 0.90F

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
    point.alt = node->coordinate.alt;
    return point;
}

gcs::path Path_planner::plan_path(point start, point goal)
{
    if(DEBUG) std::cout << "Shrinking the polygon..." << std::endl;
    shrink_polygon(SHRINK_FACTOR);
    nodes.push_back(Node(start, -1));
    nodes.push_back(Node(goal, -2));

    //std::cout << "Is start point inside: " << fence->point_legal(start, UTM) << std::endl;

    if(DEBUG) std::cout << "Connecting all nodes..." << std::endl;
    connect_all_nodes();    // Connect all nodes with each other

    // Run a-star from start to goal
    if(DEBUG) std::cout << "A* pathplanning is started..." << std::endl;
    std::vector<Node*> path_nodes = a_star(&nodes[nodes.size() - 2], &nodes.back());
    if(DEBUG) std::cout << "A* pathplanning is finished. Printing the path..." << std::endl;

    // Convert nodes into a path of waypoints
    gcs::path waypoint_path;
    for(unsigned int i = 0; i<path_nodes.size(); i++)
    {
        if(DEBUGGING) path_nodes[i]->print_node();
        waypoint_path.path.push_back(convert_node_to_waypoint(path_nodes[i]));
    }

    return waypoint_path;   // Missing: check if this works OBS fix so it returns lat and long instead of UTM
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

void Path_planner::export_as_csv(std::string filename)
{
    std::ofstream csvfile(filename);
    if (csvfile.is_open())
    {
        csvfile << "Name,Description,Latitude,Longitude\n";
        for(unsigned int i = 0; i<nodes.size(); i++)
        {
            point temp_point = nodes[i].coordinate;
            fence->UTM_to_geodetic(temp_point);
            csvfile << i+1 << ",none," << temp_point.lat << "," << temp_point.lon << "\n";
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
