#ifndef vehicle_h
#define vehicle_h

#include <cmath>
#include <vector>
#include "map.h"

class Vehicle {
public:
  struct State {
    State(double x, double y, double s, double d, double v, double yaw);
    State(std::vector<double> sensor_input);

    double x;
    double y;
    double s;
    double d;
    double v;   // in miles/h
    double yaw; // in radian
    double vx;
    double vy;
    double target_v;
    int target_lane;
  };

  Vehicle(Map map, double initial_velocity, double initial_lane, double lane_width);
  ~Vehicle();

  void UpdateTragectory(State state,
                        std::vector<double> previous_path_x,
                        std::vector<double> previous_path_y,
                        std::vector<double>& next_x_vals,
                        std::vector<double>& next_y_vals);
private:
  static const int NUMBER_OF_POINTS;
  static const int POINTS_AHEAD;
  static const double POINT_INTERVAL;
  static const double TIME_INTERVAL;
  static const double LANE_CHANGE_RATE;
  static const double ACCELERATION;

  Map map_;
  double current_velocity_;
  double current_lane_;
  const double lane_width_;
};

#endif /* vehicle_h */
