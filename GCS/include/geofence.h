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

#define DEBUG false
#define lat_index 2
#define lon_index 3
#define STEP_SIZE 0.5//0.01
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

enum coordtype_t {UTM,LatLon};

class geofence
{
public:
  geofence();
  ~geofence();
  void set_fence(std::vector<point>);
  void set_fence(std::string);
  void set_fence_csv(std::string geofence_path, bool inversed);
  void set_obstacles(std::vector<obstacle>);
  void set_obstacles(std::string);
  void set_max_altitude(double);
  bool point_legal(point test_point, coordtype_t coordtype);
  bool edge_legal(point, point);
  bool edge_legal_bin(point p1, point p2);
  bool geodetic_to_UTM(point &);
  bool UTM_to_geodetic(point &p);
  bool self_test();
  std::vector<point> get_fence();

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
