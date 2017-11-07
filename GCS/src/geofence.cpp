#include "geofence.h"

// http://www.convertcsv.com/kml-to-csv.htm

geofence::geofence()
{}

geofence::~geofence()
{}

// This function is not working. It should load the geofence from file
// It is kept just to get the idea, until we can get it from the server.
void geofence::set_fence(std::string geofence_path)
{

    std::ifstream geofence_file (geofence_path);
    std::string fieldValue, lat, lon;
    std::getline  (geofence_file,fieldValue,'\n'); // Skip first line
    int pointCount = 1;
    std::vector<point> fence_points;
    while ( geofence_file.good() && std::getline ( geofence_file, fieldValue, ',' ) )
    {
	    point temp_point;
        // Skip first field in if statement (to ensure it does not repeat last line)
        std::getline ( geofence_file, fieldValue, ',' );	// Skip second field
        std::getline ( geofence_file, lat, ',' ); 			// Read Lat
        std::getline ( geofence_file, lon, ',' ); 			// Read lon
        std::getline ( geofence_file, fieldValue, '\n' ); // Skip the rest of the line
        temp_point.lat = std::atof(lat.c_str());
        temp_point.lon = std::atof(lon.c_str());
        fence_points.push_back(temp_point);
    }
    geofence_file.close();
    set_fence(fence_points);

    if(DEBUG)
    {
        std::cout << "Loaded geofence with points: lat lon" << std::endl;
        for( int i = 0; i < fence_points.size(); i++)
	        std::cout << (i+1) << ": " << fence_points[i].lat << " " << fence_points[i].lon << std::endl;
    }
}

void geofence::set_fence_csv(std::string geofence_path, bool inversed)
{
    std::ifstream geofence_file (geofence_path);
    std::string fieldValue, lat, lon;
    std::vector<point> fence_points;

    while ( geofence_file.good())
    {
	    point temp_point;
        std::getline ( geofence_file, lat, ',' ); 			// Read Lat
        std::getline ( geofence_file, lon, '\n' ); 			// Read lon
        if(geofence_file.eof()) // Makes sure we dont read the last line twice
            break;
        if(inversed)
        {
            temp_point.lon = std::atof(lat.c_str());
            temp_point.lat = std::atof(lon.c_str());
        }
        else
        {
            temp_point.lat = std::atof(lat.c_str());
            temp_point.lon = std::atof(lon.c_str());
        }
        fence_points.push_back(temp_point);
    }
    geofence_file.close();
    set_fence(fence_points);

    if(DEBUG)
    {
        std::cout << "Loaded geofence with points: lat lon" << std::endl;
        for( int i = 0; i < fence_points.size(); i++)
	        std::cout << (i+1) << ": " << fence_points[i].lat << " " << fence_points[i].lon << std::endl;
    }
}

void geofence::set_obstacles(std::string obstacle_path)
{
    std::ifstream obstacle_file (obstacle_path);
    std::string height, pointsInObstacleStr, lat, lon, fieldValue;
    std::vector<obstacle> temp_obstacles;
    while( obstacle_file.good() && std::getline( obstacle_file, height, ',' ) )
    {
        std::getline( obstacle_file, pointsInObstacleStr, '\n' );
        int pointsInObstacle = std::atoi(pointsInObstacleStr.c_str());
        //std::cout << "PointsInObstacle: " << pointsInObstacleStr << " " << pointsInObstacle << std::endl;
        obstacle temp_obstacle;
        temp_obstacle.height = std::atof(height.c_str());
        for( int i = 0; i < pointsInObstacle; i++) // For all points in this obstacle
        {
            point temp_point;
            std::getline ( obstacle_file, fieldValue, ',' );	// Skip first field (until delimiter ,)
            std::getline ( obstacle_file, fieldValue, ',' );	// Skip second field (until delimiter ,)
            std::getline ( obstacle_file, lat, ',' ); 			// Read Lat (until delimiter ,)
            std::getline ( obstacle_file, lon, ',' ); 			// Read lon(until delimiter ,)
            std::getline ( obstacle_file, fieldValue, '\n' ); // Skip the rest of the line	(until delimiter \n)
            temp_point.lat = std::atof(lat.c_str());
            temp_point.lon = std::atof(lon.c_str());
            temp_obstacle.points.push_back(temp_point);
        }
        temp_obstacles.push_back(temp_obstacle);
    }
    obstacle_file.close();
    set_obstacles(temp_obstacles);

    if(DEBUG)
    {
        std::cout << "Loaded obstacles:" << std::endl;
        for( int i = 0; i < temp_obstacles.size(); i++)
        {
            std::cout << "Obstacle" << i << ": lat lon" << std::endl;
            std::cout << "\tHeight: " << temp_obstacles[i].height << std::endl;
            for( int j = 0; j < temp_obstacles[i].points.size(); j++)
                std::cout  << '\t' << temp_obstacles[i].points[j].lat << " " << temp_obstacles[i].points[j].lon << std::endl;
        }
    }
}

void geofence::set_fence(std::vector<point> _fence)
{
  if(USE_UTM){
    for(int i=0; i < _fence.size(); i++)
    {
      geodetic_to_UTM(_fence[i]);
    }
    fence = _fence;
  }
  //ROS_INFO("Obstacles size %i, with %i points in first",obstacles.size(),obstacles[0].points.size());
}

void geofence::set_obstacles(std::vector<obstacle> _obstacles)
{
  if(USE_UTM)
  {
    for(int i=0; i < _obstacles.size(); i++)
    {
      for(int j=0; j < _obstacles[i].points.size(); j++)
      {
        geodetic_to_UTM(_obstacles[i].points[i]);
      }
    }
  }
  obstacles = _obstacles;
}

void geofence::set_max_altitude(double altitude)
{
  max_altitude = altitude;
}

bool geofence::point_legal(point test_point, coordtype_t coordtype)
{
  if(test_point.alt > max_altitude )
    return false;

  if(coordtype == LatLon) geodetic_to_UTM(test_point);
  bool point_ok = false;
  // Check geofence
  point_ok = inside_polygon(fence, test_point);
  // Check obstacles
  int i = 0;
  while(point_ok && i < obstacles.size())
  {
    if( inside_polygon(obstacles[i].points, test_point) ) // We are at an obstacle in the xy-plane
    {
      if( test_point.alt > obstacles[i].height) // We are above the obstacle
        point_ok = true;
      else                                  // We are inside an obstacle
        point_ok = false;
    }
    i++;
  }
  return point_ok;
}

bool geofence::edge_legal(point p1, point p2)
{
  // TODO: The pertubation needs to be done using UTM in some way
  bool result = true;
  // Make vector between points
  double vec_x = p2.lon - p1.lon;
  double vec_y = p2.lat - p1.lat;
  double vec_z = p2.alt - p1.alt;
  double length = std::sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z);
  //ROS_INFO("length: %f", length);
  double x_temp, y_temp, z_temp;
  if(length > STEP_SIZE)
  {
    int steps = length/STEP_SIZE;
    int step = 0;
    while( result == true && step <= steps) // Check points in the vector
    {
      x_temp = p1.lon + vec_x*( (double)step/(double)steps);  // Remenber to type cast!
      y_temp = p1.lat + vec_y*((double)step/(double)steps);
      z_temp = p1.alt + vec_z*((double)step/(double)steps);
      point p_temp;
      p_temp.lon = x_temp;
      p_temp.lat = y_temp;
      p_temp.alt = z_temp;
      //ROS_INFO("p_temp: %f,%f,%f",p_temp.lon,p_temp.lat,p_temp.alt);
      result = point_legal(p_temp, UTM);
      //ROS_INFO("step: %i result: %i",step,result);
      step++;
    }
  }
  else
    result = point_legal(p1, UTM) && point_legal(p2, UTM);

  return result;
}

// Binary edge collision detector
bool geofence::edge_legal_bin(point p1, point p2)
{
   // Make vector between points
   double vec_x = p2.lon - p1.lon;
   double vec_y = p2.lat - p1.lat;
   double vec_z = p2.alt - p1.alt;
   double length = std::sqrt(vec_x*vec_x + vec_y*vec_y + vec_z*vec_z);

   int n = length/STEP_SIZE;
   int levels = ceil(log2(n));

   for(int i = 1; i <= levels; i++)
   {
       int steps = pow(2.0,i-1);

       double lon_step = vec_x/steps;
       double lat_step = vec_y/steps;
       double alt_step = vec_z/steps;

       for(int j = 1; j <= steps; j++)
       {
           point p_temp;

           p_temp.lon = p1.lon + (j - 0.5)*lon_step;
           p_temp.lat = p1.lat + (j - 0.5)*lat_step;
           p_temp.alt = p1.alt + (j - 0.5)*alt_step;

           if(!point_legal(p_temp, UTM))
           {
               return false;
           }
       }
   }
   return true;
}

bool geofence::inside_polygon(std::vector<point> _fence, point test_point)
{
  bool inside_polygon = false;

  int n_points = _fence.size();
  double x_test = test_point.lon;
  double y_test = test_point.lat;

  // Find minimum and maximum values
  double x_min = _fence[0].lon;
  double x_max = _fence[0].lon;
  double y_min = _fence[0].lat;
  double y_max = _fence[0].lat;
  for(int i = 1; i < n_points; i++)
  {
    x_min = std::min(x_min,_fence[i].lon);
    x_max = std::max(x_max,_fence[i].lon);
    y_min = std::min(y_min,_fence[i].lat);
    y_max = std::max(y_max,_fence[i].lat);
  }

  // Abort further checks if the point is clearly outside
  if( x_test > x_max || x_test < x_min || y_test > y_max || y_test < y_min)
    return false;

  for(int i = 0, j = n_points -1; i < n_points; j = i++)
  {
    double x_point_i = _fence[i].lon;
    double y_point_i = _fence[i].lat;
    double x_point_j = _fence[j].lon;
    double y_point_j = _fence[j].lat;
    if ( (( y_point_i > y_test) != (y_point_j > y_test)) &&
         ( x_test < (x_point_j - x_point_i) * (y_test-y_point_i) / (y_point_j - y_point_i) + x_point_i) )
         inside_polygon = !inside_polygon;
  }
  return inside_polygon;
}

bool geofence::geodetic_to_UTM(point &p)
{
  if(DEBUG) ROS_INFO("UTM_zone: %i", UTM_zone);
  double lon0 = (((double)UTM_zone -1)*6-180+3);
  if(DEBUG) ROS_INFO("lon0: %f",lon0);
  try
  {
    GeographicLib::TransverseMercator proj(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f(), GeographicLib::Constants::UTM_k0());
    point local_point;
    if(DEBUG) ROS_INFO("geodetic: %f,%f", p.lat, p.lon);
    proj.Forward(lon0,p.lat,p.lon,local_point.lat,local_point.lon);
    local_point.lat += false_easting;
    if(DEBUG) ROS_INFO("geodetic_to_UTM: %f,%f",local_point.lat,local_point.lon);

    p = local_point;
  }
  catch (const std::exception& e){
    std::stringstream error;
    error << e.what();
    ROS_ERROR("Exception %s",error.str().c_str());
    return false;
  }
  return true;
}

bool geofence::UTM_to_geodetic(point &p)
{
  if(DEBUG) ROS_INFO("UTM_zone: %i", UTM_zone);
  double lon0 = (((double)UTM_zone -1)*6-180+3);
  if(DEBUG) ROS_INFO("lon0: %f",lon0);
  try
  {
    GeographicLib::TransverseMercator proj(GeographicLib::Constants::WGS84_a(), GeographicLib::Constants::WGS84_f(), GeographicLib::Constants::UTM_k0());
    point local_point;
    if(DEBUG) ROS_INFO("geodetic: %f,%f", p.lat, p.lon);
    p.lat -= false_easting;
    proj.Reverse(lon0,p.lat,p.lon,local_point.lat,local_point.lon);
    if(DEBUG) ROS_INFO("geodetic_to_UTM: %f,%f",local_point.lat,local_point.lon);

    p = local_point;
  }
  catch (const std::exception& e){
    std::stringstream error;
    error << e.what();
    ROS_ERROR("Exception %s",error.str().c_str());
    return false;
  }
  return true;
}

bool geofence::self_test()
{
  if(DEBUG) ROS_INFO("Starting selftest");
  if(DEBUG) ROS_INFO("Testing UTM");
  bool ret_val = test_UTM();

  std::vector<point> fence_backup;
  std::vector<obstacle> obstacles_backup;
  std::vector<point> points;
  std::vector<point> local_fence;
  std::vector<obstacle> local_obstacles;
  // Backup existing data
  if(fence.size() > 0)
  {
    fence_backup = fence;
  }
  fence.clear();
  if(obstacles.size() > 0)
  {
    obstacles_backup = obstacles;
  }
  obstacles.clear();

  // Set geofence
  point p1, p2, p3, p4, p_inside, p_outside;

  p1.lat = 55.0;
  p1.lon = 10.0;
  local_fence.push_back(p1);
  p2.lat = 55.1;
  p2.lon = 10.0;
  local_fence.push_back(p2);
  p3.lat = 55.1;
  p3.lon = 10.1;
  local_fence.push_back(p3);
  p4.lat = 55.0;
  p4.lon = 10.1;
  local_fence.push_back(p4);
  set_fence(local_fence);
  // Define testpoints
  p_inside.lat = 55.05;
  p_inside.lon = 10.05;
  p_outside.lat = 55.2;
  p_outside.lon = 10.2;

  // Perform inclusion tests
  if(DEBUG) ROS_INFO("Testing point inside");
  bool inside_test = point_legal(p_inside, LatLon);
  if(!inside_test)
    ROS_ERROR("Inside test failed");
  if(DEBUG) ROS_INFO("Testing point outside");
  bool outside_test = !point_legal(p_outside, LatLon); // NOTE: Negation becase the point should be outside
  if(!outside_test)
    ROS_ERROR("Outside test failed");

  // Restore existing data
  if(fence.size() > 0)
  {
    fence_backup = fence;
  }
  if(obstacles.size() > 0)
  {
    obstacles_backup = obstacles;
  }

  ret_val = ret_val && inside_test && outside_test;
  if(ret_val == true)
  {
    if(DEBUG) ROS_INFO("All tests passed");
  }
  return ret_val;
}

bool geofence::test_UTM()
{
  int ret_val = 0;
  point p, p_UTM_expected;
  // These coordinates are based on online converters and Kjeld Jensen's utm_test.py
  p.lat = 55.0000000000;
  p.lon = 009.0000000000;
  p_UTM_expected.lat = 500000;
  p_UTM_expected.lon = 6094791.420999;

  if( geodetic_to_UTM(p) )
  {
    if( p_UTM_expected.lat - TEST_ACCURACY < p.lat && p.lat < p_UTM_expected.lat + TEST_ACCURACY)
    {
      if(DEBUG) ROS_INFO("geodetic test: lat OK");
    }
    else
      ret_val++;
    if( p_UTM_expected.lon - TEST_ACCURACY < p.lon && p.lon < p_UTM_expected.lon + TEST_ACCURACY)
    {
      if(DEBUG) ROS_INFO("geodetic test: lon OK");
    }
    else
      ret_val++;
  }
  if(ret_val > 0)
    ROS_ERROR("geodetic test: ERROR. Expected %f,%f but got %f,%f",p_UTM_expected.lat, p_UTM_expected.lon, p.lat, p.lon);

  return (ret_val == 0);
}

std::vector<point> geofence::get_fence()
{
    return fence;
}
