#include <cmath>
#include <limits>
#include "planner.h"
#include "lane.h"

using namespace std;

const double Lane::LANE_WIDTH = 4.0; // in meters

// TODO make configurable
const double Lane::FRONT_BUFFER_START = 120;
const double Lane::FRONT_BUFFER_END = 0;
const double Lane::BACK_BUFFER_START = 15;
const double Lane::BACK_BUFFER_END = -12.5;
const double Lane::SIDE_BUFFER_START = 8;
const double Lane::SIDE_BUFFER_END = -8;
const double Lane::VELOCITY_LIMIT_BUFFER = 27;
const double Lane::STOP_BUFFER = 17;
const double Lane::LANE_MARGIN = 2.0; // 2.5;

const double Lane::Calculator::UPPER_LIMIT = 4.0;
const double Lane::Calculator::LOWER_LIMIT = -UPPER_LIMIT;

// Cost calculator implementation

Lane::Calculator::Calculator(double high, double low, double weight):
high(high),
low(low),
weight(weight) {
}

double Lane::Calculator::operator()(double value) const {
  double ratio = (value - low) / (high - low);
  auto x = LOWER_LIMIT + (ratio * (UPPER_LIMIT - LOWER_LIMIT));
  return weight / (1 + exp(x));
}

// Constructors and destructor

Lane::Lane(int id, double max_velocity, double min_velocity):
id_(id),
max_velocity_(max_velocity),
min_velocity_(min_velocity),
d_(LANE_WIDTH/2.0 + LANE_WIDTH * id),
velocity_cost_(50, 25, 5),
front_buffer_cost_(80, 30, 8),
back_buffer_cost_(15, 0, 50),
collision_cost_(-10, 0, 50) {
}

Lane::~Lane() {
}

// Public methods

void Lane::Evaluate(Vehicle::State current, std::vector<Vehicle::State> detected) {
  
  Vehicle::State* front = 0;
  Vehicle::State* back = 0;
  double min_front_buffer = numeric_limits<double>::max();
  double min_back_buffer = numeric_limits<double>::max();
  double lock_cost = 0;
  velocity_ = max_velocity_;

  for (auto& other : detected) {
    if (Detects(other)) {
      double buffer = other.s - current.s;
      if (FRONT_BUFFER_END < buffer && buffer < FRONT_BUFFER_START && buffer < min_front_buffer) {
        min_front_buffer = buffer;
        front = &other;
      }
      
      if (!Accommodates(current) && BACK_BUFFER_END < buffer && buffer < BACK_BUFFER_START && std::abs(buffer) < min_back_buffer) {
        min_back_buffer = std::abs(buffer);
        back = &other;
      }
      
      if (!Accommodates(current) && SIDE_BUFFER_END < buffer && buffer < SIDE_BUFFER_START ) {
        lock_cost = 10.0;
      }
    }
  }
  
  double front_buffer = numeric_limits<double>::max();
  double back_buffer = numeric_limits<double>::max();
  double back_velocity_difference = numeric_limits<double>::min();
  double front_velocity = max_velocity_;
  
  if (front != 0) {
    front_buffer = std::abs(front->s - current.s);
    front_velocity = front->v;
    
    if (front_buffer < VELOCITY_LIMIT_BUFFER) {
      velocity_ = front_buffer < STOP_BUFFER ? front_velocity * 0.7 : front_velocity;
    }
  }
  
  if (back != 0) {
    back_buffer = min_back_buffer;
    back_velocity_difference = back->v - current.v;
  }
  
  cost_ = lock_cost;
  cost_ += velocity_cost_(front_velocity);
  cost_ += front_buffer_cost_(front_buffer);
  cost_ += back_buffer_cost_(back_buffer);
  cost_ += collision_cost_(back_velocity_difference);
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

