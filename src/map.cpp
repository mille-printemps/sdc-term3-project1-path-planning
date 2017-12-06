#include <limits>
#include <cmath>
#include <vector>
#include "map.h"
#include "util.h"

using namespace std;
using namespace util;

// Constructors and destructor

Map::Map(vector<double> waypoints_x,
         vector<double> waypoints_y,
         vector<double> waypoints_s,
         vector<double> waypoints_dx,
         vector<double> waypoints_dy) {
  waypoints_x_ = waypoints_x;
  waypoints_y_ = waypoints_y;
  waypoints_s_ = waypoints_s;
  waypoints_dx_ = waypoints_dx;
  waypoints_dy_ = waypoints_dy;
  
  s_x.set_points(waypoints_s, waypoints_x);
  s_y.set_points(waypoints_s, waypoints_y);
  s_dx.set_points(waypoints_s, waypoints_dx);
  s_dy.set_points(waypoints_s, waypoints_dy);
}

Map::~Map() {
}

// Public methods

Map::Frenet Map::GetFrenet(double x, double y, double theta) {
  int next_wp = GetNextWaypoint(x, y, theta);

  int prev_wp = next_wp - 1;

  if(next_wp == 0) {
    prev_wp  = waypoints_x_.size() - 1;
  }

  double n_x = waypoints_x_[next_wp] - waypoints_x_[prev_wp];
  double n_y = waypoints_y_[next_wp] - waypoints_y_[prev_wp];
  double x_x = x - waypoints_x_[prev_wp];
  double x_y = y - waypoints_y_[prev_wp];

  // find the projection of x onto n
  double proj_norm = (x_x * n_x + x_y * n_y) / (n_x * n_x + n_y * n_y);
  double proj_x = proj_norm * n_x;
  double proj_y = proj_norm * n_y;

  double frenet_d = Distance(x_x, x_y, proj_x, proj_y);

  //see if d value is positive or negative by comparing it to a center point
  double center_x = 1000 - waypoints_x_[prev_wp];
  double center_y = 2000 - waypoints_y_[prev_wp];
  double centerToPos = Distance(center_x, center_y, x_x, x_y);
  double centerToRef = Distance(center_x, center_y, proj_x, proj_y);

  if(centerToPos <= centerToRef) {
    frenet_d *= -1;
  }

  // calculate s value
  double frenet_s = 0;
  for(int i = 0; i < prev_wp; i++) {
    frenet_s += Distance(waypoints_x_[i], waypoints_y_[i], waypoints_x_[i+1], waypoints_y_[i+1]);
  }

  frenet_s += Distance(0, 0 , proj_x, proj_y);

  Frenet frenet(frenet_s, frenet_d);
  return frenet;
}

Map::Point Map::GetXY(double s, double d) {
  return {s_x(s) + d * s_dx(s), s_y(s) + d * s_dy(s)};
}

// Private methods

int Map::GetClosestWaypoint(double x, double y) {
  double closestLen = numeric_limits<double>::max();
  int closestWaypoint = 0;

  for(int i = 0; i < waypoints_x_.size(); i++) {
    double map_x = waypoints_x_[i];
    double map_y = waypoints_y_[i];
    double dist = Distance(x, y, map_x ,map_y);

    if(dist < closestLen) {
      closestLen = dist;
      closestWaypoint = i;
    }
  }

  return closestWaypoint;
}

int Map::GetNextWaypoint(double x, double y, double theta) {
  int closestWaypoint = GetClosestWaypoint(x, y);

  double map_x = waypoints_x_[closestWaypoint];
  double map_y = waypoints_y_[closestWaypoint];

  double heading = atan2(map_y - y, map_x - x);
  double angle = abs(theta - heading);

  if(angle > Pi() / 4) {
    closestWaypoint++;
  }

  return closestWaypoint;
}
