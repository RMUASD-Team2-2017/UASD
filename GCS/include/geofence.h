#pragma once

// General ROS
#include <ros/ros.h>

// General
#include <iostream>
#include <string>
#include <fstream>
#include <algorithm>
#include <iterator>
#include <stdlib.h>
#include <vector>

// Geographic lib
#include <exception>
#include <sstream>
#include <iomanip>
//#include <GeographicLib/TransverseMercatorExact.hpp>
#include <GeographicLib/TransverseMercator.hpp>

#define DEBUG true
#define lat_index 2
#define lon_index 3
#define STEP_SIZE 0.01
#define USE_UTM true
#define TEST_ACCURACY 0.1

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
  void set_fence(std::vector<point>);
  void set_fence(std::string);
  void set_obstacles(std::vector<obstacle>);
  void set_obstacles(std::string);
  void set_max_altitude(double);
  bool point_legal(point);
  bool edge_legal(point, point);
  bool geodetic_to_UTM(point &);
  bool self_test();

private:
  bool inside_polygon(std::vector<point> _fence, point test_point);
  bool test_UTM();
  //split(const std::string& str, std::vector<std::string>& cont, char delim = ' ')
  std::vector<point> fence;
  std::vector<obstacle> obstacles;
  double max_altitude;
  int UTM_zone = 32;
  double false_easting = 500000;
};
