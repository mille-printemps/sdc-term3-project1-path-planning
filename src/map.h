#ifndef map_h
#define map_h

#include <vector>
#include "spline.h"

class Map {
public:
  // Public struct
  struct Point {
    Point(double x, double y): x(x), y(y){}
    double x;
    double y;
  };

  struct Frenet {
    Frenet(double s, double d): s(s), d(d){}
    double s;
    double d;
  };

  // Constructor and destructor
  Map(std::vector<double> waypoints_x,
      std::vector<double> waypoints_y,
      std::vector<double> waypoints_s,
      std::vector<double> waypoints_dx,
      std::vector<double> waypoints_dy);
  ~Map();

  // Public methods
  Frenet GetFrenet(double x, double y, double theta);
  Point GetXY(double s, double d);

private:
  // Private variables
  std::vector<double> waypoints_x_;
  std::vector<double> waypoints_y_;
  std::vector<double> waypoints_s_;
  std::vector<double> waypoints_dx_;
  std::vector<double> waypoints_dy_;
  
  tk::spline s_x;
  tk::spline s_y;
  tk::spline s_dx;
  tk::spline s_dy;

  // Private methods
  int GetClosestWaypoint(double x, double y);
  int GetNextWaypoint(double x, double y, double theta);
};

#endif /* map_h */
