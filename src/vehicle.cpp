#include <cmath>
#include <vector>
#include "spline.h"
#include "vehicle.h"
#include "util.h"

using namespace std;
using namespace util;

// Constants

const int Vehicle::NUMBER_OF_POINTS = 50;
const int Vehicle::POINTS_AHEAD = 3;
const double Vehicle::POINT_INTERVAL = 35.0; // in meters
const double Vehicle::TIME_INTERVAL = 0.02;  // in seconds
const double Vehicle::LANE_CHANGE_RATE = 0.01;
const double Vehicle::ACCELERATION = 0.28;

// Constructors and destructor

Vehicle::State::State(double x, double y, double s, double d, double v, double yaw):
  x(x),
  y(y),
  s(s),
  d(d),
  v(v),
  yaw(Deg2rad(yaw)),
  vx(v * cos(Deg2rad(yaw))),
  vy(v * sin(Deg2rad(yaw))),
  target_v(v),
  target_lane(1) {
}

Vehicle::State::State(vector<double> sensor_input) {
  x = sensor_input[1];
  y = sensor_input[2];
  vx = MetersPerSecond2MilesPerHour(sensor_input[3]);
  vy = MetersPerSecond2MilesPerHour(sensor_input[4]);
  s = sensor_input[5];
  d = sensor_input[6];
  v = sqrt(vx * vx + vy * vy);
  yaw = atan2(vy, vx);
  target_v = v;
  target_lane = 1;
}

Vehicle::Vehicle(Map map, double initial_velocity, double initial_lane, double lane_width):
map_(map),
current_velocity_(initial_velocity),
current_lane_(initial_lane),
lane_width_(lane_width) {
}

Vehicle::~Vehicle() {
}

// Public methods

void Vehicle::UpdateTragectory(State state,
                               vector<double> previous_path_x,
                               vector<double> previous_path_y,
                               vector<double>& next_x_vals,
                               vector<double>& next_y_vals) {

  int previous_size = previous_path_x.size();
  
  // Points from the starting reference point in the global map coordinate
  vector<double> points_x;
  vector<double> points_y;
  
  // x, y and yaw at the starting reference point in the global map coordinate
  double reference_x;
  double reference_y;
  double reference_yaw;
  
  // Augment the points x,y depending on the number of the previous x,y
  if (previous_size < 2) {
    reference_x = state.x;
    reference_y = state.y;
    reference_yaw = state.yaw;
    
    points_x.push_back(state.x - cos(reference_yaw));
    points_x.push_back(state.x);
    
    points_y.push_back(state.y - sin(reference_yaw));
    points_y.push_back(state.y);
  } else {
    reference_x = previous_path_x[previous_size-1];
    reference_y = previous_path_y[previous_size-1];
    
    double previous_x = previous_path_x[previous_size-2];
    double previous_y = previous_path_y[previous_size-2];
    
    reference_yaw = atan2(reference_y - previous_y, reference_x - previous_x);
    
    points_x.push_back(previous_x);
    points_x.push_back(reference_x);
    
    points_y.push_back(previous_y);
    points_y.push_back(reference_y);
  }
  
  // Add POINTS_AHEAD points at POINT_INTERVAL ahead of the reference
  for (int i = 1; i <= POINTS_AHEAD; i++ ) {
    Map::Point next_waypoint = map_.GetXY(state.s + POINT_INTERVAL * i, lane_width_/2 + lane_width_ * current_lane_);
    
    points_x.push_back(next_waypoint.x);
    points_y.push_back(next_waypoint.y);
  }
  
  // Convert into local coordinates by shifting the angle to 0 degrees.
  for (int i = 0; i < points_x.size(); i++) {
    double delta_x = points_x[i] - reference_x;
    double delta_y = points_y[i] - reference_y;
    points_x[i] = delta_x * cos(-reference_yaw) - delta_y * sin(-reference_yaw);
    points_y[i] = delta_x * sin(-reference_yaw) + delta_y * cos(-reference_yaw);
  }
  
  // Fill the next x,y with the previous x,y for now
  for (int i = 0; i < previous_size; i++) {
    next_x_vals.push_back(previous_path_x[i]);
    next_y_vals.push_back(previous_path_y[i]);
  }
  
  tk::spline spline;
  spline.set_points(points_x, points_y);
  
  // Calculate points on the spline so that the vehicle can run at the reference velocity
  double target_x = POINT_INTERVAL;
  double target_y = spline(target_x);
  double target_distance = Distance(target_x, target_y, 0, 0);
  
  double x_add_on = 0;
  
  // Fill the rest of the path planner after filling it with previous points, keeping NUMBER_OF_POINTS points
  for (int i = 0; i < NUMBER_OF_POINTS - previous_size; i++) {
    current_velocity_ += current_velocity_ < state.target_v ? ACCELERATION : -ACCELERATION;
    current_lane_ += current_lane_ < state.target_lane ? LANE_CHANGE_RATE : -LANE_CHANGE_RATE;
    
    double N = target_distance / (TIME_INTERVAL * MilesPerHour2MetersPerSecond(current_velocity_));
    double x_point = x_add_on + target_x / N;
    double y_point = spline(x_point);
    
    x_add_on = x_point;
    
    double x_local = x_point;
    double y_local = y_point;
    
    // convert into the global map coordinates
    x_point = x_local * cos(reference_yaw) - y_local * sin(reference_yaw);
    y_point = x_local * sin(reference_yaw) + y_local * cos(reference_yaw);
    x_point += reference_x;
    y_point += reference_y;
    
    next_x_vals.push_back(x_point);
    next_y_vals.push_back(y_point);
  }
}
