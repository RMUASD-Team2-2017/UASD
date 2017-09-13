#pragma once

#include <ros/ros.h>
#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <stdlib.h>
#include <vector>

#define lat_index 2
#define lon_index 3
#define STEP_SIZE 0.01

struct point{
  double lat;
  double lon;
  double alt;
};

struct obstacle{
  std::vector<point> points;
  double height;
};

class geofence
{
public:
  geofence();
  ~geofence();
  bool set(std::string, std::string);
  void set_fence(std::vector<point>);
  void set_obstacles(std::vector<obstacle>);
  void set_max_altitude(double);
  bool point_legal(point);
  bool edge_legal(point, point);
  bool inside_polygon(std::vector<point> _fence, point test_point);
private:
  //split(const std::string& str, std::vector<std::string>& cont, char delim = ' ')

  std::vector<point> fence;
  std::vector<obstacle> obstacles;
  double max_altitude;
};
