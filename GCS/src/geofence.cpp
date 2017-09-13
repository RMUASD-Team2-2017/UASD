#include "geofence.h"

// http://www.convertcsv.com/kml-to-csv.htm

geofence::geofence()
{}

geofence::~geofence()
{}

// This function is not working. It should load the geofence from file
bool geofence::set(std::string geofence_path, std::string obstacle_path)
{
  std::ifstream geofence_file ("/home/mathias/Downloads/Geofence.csv");//geofence_path);
  if(geofence_file.is_open())
  {
    std::string line;
    while(getline(geofence_file,line))
    {

      // http://www.martinbroadhurst.com/how-to-split-a-string-in-c.html
      std::vector<std::string> splitted;
      //std::istringstream iss(line);
      //split(line,splitted);
      //std::copy(std::istream_iterator<std::string>(iss),
      //std::istream_iterator<std::string>(),
      //std::back_inserter(splitted));
      point p;
      //std::string str = splitted[lat_index];
      //p.lat = std::atof(str.c_str());
      //p.lon = std::atof(splitted[lon_index].c_str());
      //fence.push_back(p);

      ROS_INFO("%i",splitted.size());
      //ROS_INFO("%s",splitted[lat_index]);
    }


    geofence_file.close();
    return true;
  }
  else
  {
    ROS_INFO("Not open");
    return false;
  }
}

// void geofence::split(const std::string& str, std::vector<std::string>& cont, char delim = ' ')
// {
//     std::stringstream ss(str);
//     std::string token;
//     while (std::getline(ss, token, delim)) {
//         cont.push_back(token);
//     }
// }

void geofence::set_fence(std::vector<point> _fence)
{
  fence = _fence;
  //ROS_INFO("Obstacles size %i, with %i points in first",obstacles.size(),obstacles[0].points.size());
}

void geofence::set_obstacles(std::vector<obstacle> _obstacles)
{
  obstacles = _obstacles;
}

void geofence::set_max_altitude(double altitude)
{
  max_altitude = altitude;
}

bool geofence::point_legal(point test_point)
{
  if(test_point.alt > max_altitude )
    return false;

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
      result = point_legal(p_temp);
      //ROS_INFO("step: %i result: %i",step,result);
      step++;
    }
  }
  else
    result = point_legal(p1) && point_legal(p2);

  return result;
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
