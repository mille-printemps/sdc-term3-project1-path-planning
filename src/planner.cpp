#include <iostream>
#include <cmath>
#include <vector>
#include "planner.h"
#include "vehicle.h"
#include "map.h"
#include "lane.h"

using namespace std;

// Constants

const double Planner::MAX_VELOCITY = 49.0;
const double Planner::MIN_VELOCITY = 30.0;
const double Planner::INITIAL_LANE = 1;
  
const double Planner::MIN_LANE_CHANGE_COST_DIFFERENCE = 0.5;
const double Planner::MAX_LANE_CHANGE_COST = 5.5;
const double Planner::MIN_LANE_CHANGE_VELOCITY = 35.0;

// Constructors and destructor

Planner::Planner(Map map):
vehicle_(map, MAX_VELOCITY, INITIAL_LANE, Lane::LANE_WIDTH),
lane_one_(0, MAX_VELOCITY, MIN_VELOCITY),
lane_two_(1, MAX_VELOCITY, MIN_VELOCITY),
lane_three_(2, MAX_VELOCITY, MIN_VELOCITY) {
  lane_one_.NextTo(&lane_two_);
  lane_two_.NextTo(&lane_one_);
  lane_two_.NextTo(&lane_three_);
  lane_three_.NextTo(&lane_two_);
  current_lane_ = &lane_two_;
  target_lane_ = 0;
}

Planner::~Planner() {
}

// Public methods

void Planner::Maneuver(Vehicle::State current,
                       vector<vector<double> > sensor_fusion,
                       vector<double> previous_path_x,
                       vector<double> previous_path_y,
                       vector<double>& next_x_vals,
                       vector<double>& next_y_vals) {
  
  vector<Vehicle::State> sensor_inputs = GetSensorInputs(sensor_fusion);
  EvaluateSensorInputs(current, sensor_inputs);
  MayChangeLane(current, sensor_inputs);
  UpdateTrajectory(current, sensor_inputs, previous_path_x, previous_path_y, next_x_vals, next_y_vals);
}

// Private methods

vector<Vehicle::State> Planner::GetSensorInputs(vector<vector<double> > sensor_fusion){
  vector<Vehicle::State> sensor_inputs;
  for(auto& sensor_input : sensor_fusion) {
    Vehicle::State state(sensor_input);
    sensor_inputs.push_back(state);
  }
  return sensor_inputs;
}

void Planner::EvaluateSensorInputs(Vehicle::State current, vector<Vehicle::State> sensor_inputs) {
  current_lane_->Evaluate(current, sensor_inputs);
  for (auto lane : current_lane_->adjacencies()) {
    lane->Evaluate(current, sensor_inputs);
  }
}

void Planner::MayChangeLane(Vehicle::State current, vector<Vehicle::State> sensor_inputs) {
  // Check if the vehicle has moved to the lane
  if (target_lane_ != 0 && target_lane_->Accommodates(current)) {
    current_lane_ = target_lane_;
    target_lane_ = 0;
    return;
  }

  // Check if the vehicle can move to a lower cost lane
  Lane* min_cost_lane = 0;
  for (auto lane : current_lane_->adjacencies()) {
    if (min_cost_lane == 0 || lane->cost() < min_cost_lane->cost()) {
      min_cost_lane = lane;
    }
  }

  // For debugging
  cout << current_lane_->cost() << " : " << min_cost_lane->cost() << " : " << current_lane_->cost() - min_cost_lane->cost() << endl;
  cout << current.v << endl;
  cout << endl;

  if (min_cost_lane->isSafe(current, sensor_inputs) &&
      min_cost_lane->cost() < MAX_LANE_CHANGE_COST &&
      MIN_LANE_CHANGE_COST_DIFFERENCE  <= current_lane_->cost() - min_cost_lane->cost() &&
      MIN_LANE_CHANGE_VELOCITY < current.v) {
    target_lane_ = min_cost_lane;
  }
}

void Planner::UpdateTrajectory(Vehicle::State current,
                               vector<Vehicle::State> sensor_inputs,
                               vector<double> previous_path_x,
                               vector<double> previous_path_y,
                               vector<double>& next_x_vals,
                               vector<double>& next_y_vals) {
  // Update the trajectory of the vehicle with the new velocity and new lane
  current.target_v = target_lane_ == 0 ? current_lane_->velocity() : current_lane_->velocity() * 0.3 + target_lane_->velocity() * 0.7;
  current.target_lane = target_lane_ == 0 ? current_lane_->id() : target_lane_->id();
  
  vehicle_.UpdateTragectory(current, previous_path_x, previous_path_y, next_x_vals, next_y_vals);
}


