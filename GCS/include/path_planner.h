#pragma once
#include <vector>
#include <limits>
#include <string>
#include <fstream>
#include "geofence.h"

// ROS
#include <ros/ros.h>
#include "gcs/planPath.h"
#include "gcs/waypoint.h"
#include "gcs/path.h"

struct Node
{
    Node(){}

    Node(point coordinate)
    {
        this->coordinate = coordinate;
    }

    Node(point coordinate, int node_id)
    {
        this->coordinate = coordinate;
        this->node_id = node_id;
    }

    void print_node()
    {
        std::cout << "Node id: " << node_id << std::endl;
        std::cout << "Lat: " << coordinate.lat << " Lon: " << coordinate.lon << std::endl;
        std::cout << "Neighbors: " << neighbors.size() << std::endl;
    }
    int node_id;
    point coordinate;
    Node *parent = nullptr;
    std::vector<Node*> neighbors;
    double gscore = std::numeric_limits<double>::max();;
    double fscore = std::numeric_limits<double>::max();;
};

struct Vect
{
    Vect(){}
    Vect(double x, double y)
    {
        this->x = x;
        this->y = y;
    }

    double operator*(const Vect b)
    {
        return this->x * b.x + this->y * b.y;
    }

    Vect operator+(const Vect b)
    {
        return Vect(this->x + b.x, this->y + b.y);
    }

    double length()
    {
        return std::sqrt(x*x + y*y);
    }

    Vect unit_vect()
    {
        return Vect(x/length(), y/length());
    }

    double x;
    double y;
    double z;
};

class Path_planner
{
    public:
        Path_planner(geofence &fence);
        gcs::path plan_path(point start, point goal);
        void set_nodes(std::vector<point> points);
        void print_nodes();
        void export_as_csv(std::string filename);

    private:
        void connect_all_nodes();
        void shrink_polygon(double shrink_factor);
        void shrink_polygon2(double shrink_factor);
        std::vector<Node*> a_star(Node *start, Node *goal);
        double heuristic_dist(Node *start, Node *goal);
        Node* get_node_with_lowest_fscore(std::vector<Node*> &openset);
        std::vector<Node*> reconstruct_path(Node *current);
        void remove_node(std::vector<Node*> &openset, Node* node);
        bool is_node_in_set(std::vector<Node*> set, Node* node);
        gcs::waypoint convert_node_to_waypoint(Node *node);
        geofence *fence;
        std::vector<Node> nodes;
};
