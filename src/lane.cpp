#include <cmath>
#include <limits>
#include "planner.h"
#include "lane.h"

using namespace std;

const double Lane::LANE_WIDTH = 4.0; // in meters

const double Lane::FRONT_BUFFER_START = 120.0;
const double Lane::FRONT_BUFFER_END = 0.0;
const double Lane::BACK_BUFFER_START = 30.;
const double Lane::BACK_BUFFER_END = -12.5;
const double Lane::SIDE_BUFFER_START = 8.0;
const double Lane::SIDE_BUFFER_END = -8.0;
const double Lane::PREFERRED_DISTANCE_BUFFER = 27.0;
const double Lane::VELOCITY_WARNING_BUFFER = 15.0;
const double Lane::LANE_MARGIN = 2.5;
const double Lane::SAFE_DISTANCE_BUFFER = 27.0;
const double Lane::SAFE_WIDTH_BUFFER = 3.0;
const double Lane::BLOCK_COST = 20.0;

const double Lane::Calculator::UPPER_LIMIT = 6.0;
const double Lane::Calculator::LOWER_LIMIT = -UPPER_LIMIT;

// Cost calculator implementation

Lane::Calculator::Calculator(double high, double low, double weight):
high(high),
low(low),
weight(weight) {
}

double Lane::Calculator::operator()(double value) const {
  double ratio = (high - value) / (high - low);
  auto x = LOWER_LIMIT + (ratio * (UPPER_LIMIT - LOWER_LIMIT));
  return weight / (1 + exp(-x));
}

// Constructors and destructor

Lane::Lane(int id, double max_velocity, double min_velocity):
id_(id),
max_velocity_(max_velocity),
min_velocity_(min_velocity),
d_(LANE_WIDTH/2.0 + LANE_WIDTH * id),
velocity_cost_(50, 25, 5),
front_buffer_cost_(70, 10, 10),
side_back_buffer_cost_(30, 0, 90),
collision_cost_(0, -10, 60) {
}

Lane::~Lane() {
}

// Public methods

void Lane::Evaluate(Vehicle::State current, std::vector<Vehicle::State> sensor_inputs) {
  Vehicle::State* front = 0;
  Vehicle::State* side_back = 0;
  double min_front_buffer = numeric_limits<double>::max();
  double min_back_buffer = numeric_limits<double>::max();
  double block_cost = 0;
  velocity_ = max_velocity_;

  // Check status of the other vehicles on the same lane and the other lanes.
  for (auto& other : sensor_inputs) {
    if (Detects(other)) {
      double buffer = other.s - current.s;
      if (FRONT_BUFFER_END < buffer && buffer < FRONT_BUFFER_START && buffer < min_front_buffer) {
        min_front_buffer = buffer;
        front = &other;
      }
      
      if (!Detects(current) && BACK_BUFFER_END < buffer && buffer < BACK_BUFFER_START && std::abs(buffer) < min_back_buffer) {
        min_back_buffer = std::abs(buffer);
        side_back = &other;
      }
      
      if (!Detects(current) && SIDE_BUFFER_END < buffer && buffer < SIDE_BUFFER_START ) {
        block_cost = BLOCK_COST;
      }
    }
  }
  
  // Use the default values to calculate the cost if there are no vehicles ahead and behind within the range
  double front_buffer = numeric_limits<double>::max();
  double side_back_buffer = numeric_limits<double>::max();
  double back_velocity_difference = numeric_limits<double>::min();
  double front_velocity = max_velocity_;
  
  if (front != 0) {
    front_velocity = front->v;
    front_buffer = std::abs(front->s - current.s);
    
    if (front_buffer < PREFERRED_DISTANCE_BUFFER) {
      velocity_ = front_buffer < VELOCITY_WARNING_BUFFER ? front_velocity * 0.7 : front_velocity;
    }
  }
  
  if (side_back != 0) {
    side_back_buffer = min_back_buffer;
    back_velocity_difference = side_back->v - current.v;
  }
  
  cost_ = block_cost;
  cost_ += velocity_cost_(front_velocity);
  cost_ += front_buffer_cost_(front_buffer);
  cost_ += side_back_buffer_cost_(side_back_buffer);
  cost_ += collision_cost_(back_velocity_difference);
}

bool Lane::isSafe(Vehicle::State current, vector<Vehicle::State> sensor_inputs) {
  // Does a sanitary check to see if the vehicle can move to the lane
  bool safe = true;
  for (auto& other : sensor_inputs) {
    if (Accommodates(other)) {
      if (SAFE_DISTANCE_BUFFER < abs(other.s - current.s)) {
        continue;
      }

      safe = false;
      break;
    }
  }
  return safe;
}

void Lane::NextTo(Lane* lane) {
  adjacencies_.push_back(lane);
}

bool Lane::Accommodates(Vehicle::State state) const {
  return abs(state.d - d_) < 0.5;
}

int Lane::id() {
  return id_;
}

double Lane::cost() {
  return cost_;
}

double Lane::velocity() {
  return velocity_;
}

std::vector<Lane*> Lane::adjacencies() {
  return adjacencies_;
}

// Private methods

bool Lane::Detects(Vehicle::State state) const {
  return abs(state.d - d_) < LANE_MARGIN;
}

