#ifndef planner_h
#define planner_h

#include <vector>
#include "lane.h"
#include "vehicle.h"
#include "map.h"

class Planner {
public:
  Planner(Map map);
  ~Planner();

  void Maneuver(Vehicle::State current,
                std::vector<std::vector<double> > sensor_fusion,
                std::vector<double> previous_path_x,
                std::vector<double> previous_path_y,
                std::vector<double>& next_x_vals,
                std::vector<double>& next_y_vals);

private:
  static const double MIN_LANE_CHANGE_COST_DIFFERENCE;
  static const double MAX_LANE_CHANGE_COST;
  static const double MIN_LANE_CHANGE_VELOCITY;
  static const double MAX_VELOCITY;
  static const double MIN_VELOCITY;
  static const double INITIAL_LANE;

  Vehicle vehicle_;
  Lane lane_one_;
  Lane lane_two_;
  Lane lane_three_;
  Lane* current_lane_;
  Lane* target_lane_;

  std::vector<Vehicle::State> GetSensorInputs(std::vector<std::vector<double> > sensor_fusion);
  void EvaluateSensorInputs(Vehicle::State current, std::vector<Vehicle::State> sensor_inputs);
  void MayChangeLane(Vehicle::State current, std::vector<Vehicle::State> sensor_inputs);
  void UpdateTrajectory(Vehicle::State current,
                        std::vector<Vehicle::State> sensor_inputs,
                        std::vector<double> previous_path_x,
                        std::vector<double> previous_path_y,
                        std::vector<double>& next_x_vals,
                        std::vector<double>& next_y_vals);
};

#endif /* planner_h */
