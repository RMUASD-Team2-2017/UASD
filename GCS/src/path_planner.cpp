#include "path_planner.h"
#include <math.h>
#include <algorithm>
#include <iostream>

#define DEBUGGING 1
#define SHRINK_FACTOR 0.70F

Path_planner::Path_planner(geofence &fence)
{
    this->fence = &fence;
}

// Missing: Change this function to fit the geofence so that it gets the points from there.
void Path_planner::set_nodes(std::vector<point> points)
{
    for(unsigned int i = 0; i<points.size(); i++)
    {
        nodes.push_back(Node(points[i]));
    }
}

// Missing: Assumes that the coordinates from the geofence are UTM? Should the altitude be included?
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

void Path_planner::remove_node(std::vector<Node*> openset, Node* node)
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
    for(unsigned int i = 0; i<nodes.size(); i++)
    {
        for(unsigned int j = 0; j<nodes.size(); j++)
        {
            //std::cout << "legal neighbors: " << fence->edge_legal(nodes[i].coordinate, nodes[j].coordinate) << std::endl;
            if(i != j && fence->edge_legal(nodes[i].coordinate, nodes[j].coordinate))   // Missing: Virker edge_legal?
            {
                nodes[i].neighbors.push_back(&nodes[j]);
            }
        }
    }
}

std::vector<Node*> Path_planner::reconstruct_path(Node *current)
{
    std::vector<Node*> final_path = {current};
    while(final_path.back()->parent != nullptr)
    {
        final_path.push_back(final_path.back()->parent);
    }
    return final_path;
}

std::vector<Node*> Path_planner::a_star(Node *start, Node *goal)
{
    std::vector<Node*> closedset;
    std::vector<Node*> openset = {start};

    start->gscore = 0;
    start->fscore = heuristic_dist(start, goal);
    if(DEBUGGING) std::cout << "Start lat: " << start->coordinate.lat << " Start lon: " << start->coordinate.lon << std::endl;
    if(DEBUGGING) std::cout << "Goal lat: " << goal->coordinate.lat << " Goal lon: " << goal->coordinate.lon << std::endl;
    if(DEBUGGING) std::cout << "Distance: " << start->fscore << std::endl;

    while(openset.size() != 0)
    {
        Node *current = get_node_with_lowest_fscore(openset);
        if(current == goal)
            return reconstruct_path(current);

        remove_node(openset, current);
        closedset.push_back(current);

        if(DEBUGGING) current->print_node();
        if(DEBUGGING) std::cout << "Number of neighbors: " << current->neighbors.size() << std::endl;

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
        if(DEBUGGING) std::cout << "Failed" << std::endl;
        return std::vector<Node*>();    // Returns empty vector when pathplanning fails
        // Missing make sure this works
    }

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
    typedef std::numeric_limits< double > dbl;  // DEBUGGING
    std::cout.precision(dbl::max_digits10);
    print_nodes();
    shrink_nodes(SHRINK_FACTOR);
    std::cout << "\n";
    print_nodes();
    //nodes.push_back(Node(start));
    //nodes.push_back(Node(goal));

    /*
    fence->UTM_to_geodetic(nodes[0].coordinate);
    fence->UTM_to_geodetic(nodes[1].coordinate);
    std::cout << "Point1: " << nodes[0].coordinate.lat << " " << nodes[0].coordinate.lon << std::endl;
    std::cout << "Point2: " << nodes[1].coordinate.lat << " " << nodes[1].coordinate.lon << std::endl;
    std::cout << "Point legal: " << fence->point_legal(nodes[0].coordinate) << std::endl;
    std::cout << "Point legal: " << fence->point_legal(nodes[1].coordinate) << std::endl;*/
    std::cout << "DEBUGGING edge legal: " << fence->edge_legal(nodes[0].coordinate, nodes.back().coordinate) << std::endl;

    connect_all_nodes();    // Connect all nodes with each other

    // Run a-star from start to goal
    std::vector<Node*> path_nodes = a_star(&nodes[nodes.size() - 2], &nodes.back());

    // Convert nodes into a path of waypoints
    gcs::path waypoint_path;
    for(unsigned int i = 0; i<path_nodes.size(); i++)
    {
        waypoint_path.path.push_back(convert_node_to_waypoint(path_nodes[i]));
    }

    return waypoint_path;   // Missing: check if this works OBS fix so it returns lat and long instead of UTM
}

void Path_planner::shrink_nodes(double shrink_factor)
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
    typedef std::numeric_limits< double > dbl;  // DEBUGGING
    std::cout.precision(dbl::max_digits10);
    std::cout << "centroid: " << centroid.lat << " " << centroid.lon << std::endl;
    */
    // Translate to origin, scale/shrink, translate back
    for(unsigned int i = 0; i<nodes.size(); i++)
    {
        nodes[i].coordinate.lat = (nodes[i].coordinate.lat - centroid.lat)*shrink_factor + centroid.lat;
        nodes[i].coordinate.lon = (nodes[i].coordinate.lon - centroid.lon)*shrink_factor + centroid.lon;
        /*std::cout << "1: " << centroid.lon << std::endl;
        std::cout << "2: " << nodes[i].coordinate.lon << std::endl;
        std::cout << "3: " << (nodes[i].coordinate.lon - centroid.lon) << std::endl;*/
    }
}

void Path_planner::print_nodes()
{
    for(unsigned int i = 0; i<nodes.size(); i++)
    {
        std::cout << "lat: " << nodes[i].coordinate.lat << std::endl;
        std::cout << "lon: " << nodes[i].coordinate.lon << std::endl;
    }
}
