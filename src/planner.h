#ifndef planner_h
#define planner_h

#include <vector>
#include "lane.h"
#include "vehicle.h"
#include "map.h"

class Planner {
public:
  // Constructor and destructor
  Planner(Map map);
  ~Planner();

  // Public methods
  void Maneuver(Vehicle::State current,
                std::vector<std::vector<double> > sensor_fusion,
                std::vector<double> previous_path_x,
                std::vector<double> previous_path_y,
                std::vector<double>& next_x_vals,
                std::vector<double>& next_y_vals);

private:
  // Private constants
  static const double MIN_LANE_CHANGE_COST_DIFFERENCE;
  static const double MAX_LANE_CHANGE_COST;
  static const double MIN_LANE_CHANGE_VELOCITY;
  static const double SAFE_DISTANCE_BUFFER;
  static const double SAFE_WIDTH_BUFFER;
  static const double MAX_VELOCITY;
  static const double MIN_VELOCITY;
  static const double INITIAL_LANE;

  // Private variables
  Vehicle vehicle_;
  Lane lane_one_;
  Lane lane_two_;
  Lane lane_three_;
  Lane* current_lane_;
  Lane* target_lane_;

  // Private methods
  std::vector<Vehicle::State> GetSensorInputs(std::vector<std::vector<double> > sensor_fusion);
  void EvaluateSensorInputs(Vehicle::State current, std::vector<Vehicle::State> sensor_inputs);
  void MayChangeLane(Vehicle::State current);
  void UpdateTrajectory(Vehicle::State current,
                        std::vector<Vehicle::State> sensor_inputs,
                        std::vector<double> previous_path_x,
                        std::vector<double> previous_path_y,
                        std::vector<double>& next_x_vals,
                        std::vector<double>& next_y_vals);
  double MaySlowDown(Vehicle::State current, std::vector<Vehicle::State> sensor_inputs);
};

#endif /* planner_h */
