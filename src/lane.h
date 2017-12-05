#ifndef lane_h
#define lane_h

#include <vector>
#include "vehicle.h"

class Lane {
public:
  static const double LANE_WIDTH; // in meters

  Lane(int id, double max_velocity, double min_velocity);
  ~Lane();

  void Evaluate(Vehicle::State current, std::vector<Vehicle::State> detected);
  void NextTo(Lane* lane);
  bool Accommodates(Vehicle::State state) const;

  int id();
  double cost();
  double velocity();
  std::vector<Lane*> adjacencies();

private:
  struct Calculator {
    static const double UPPER_LIMIT;
    static const double LOWER_LIMIT;

    double high;
    double low;
    double weight;

    Calculator(double low, double high, double weight);
    double operator()(double value) const;
  };

  static const double FRONT_BUFFER_START;
  static const double FRONT_BUFFER_END;
  static const double BACK_BUFFER_START;
  static const double BACK_BUFFER_END;
  static const double SIDE_BUFFER_START;
  static const double SIDE_BUFFER_END;
  static const double PREFERRED_DISTANCE_BUFFER;
  static const double VELOCITY_WARNING_BUFFER;
  static const double LANE_MARGIN;

  int id_;
  double max_velocity_;
  double min_velocity_;
  
  double d_;
  double cost_;
  double velocity_;
  std::vector<Lane*> adjacencies_;

  Calculator velocity_cost_;
  Calculator front_buffer_cost_;
  Calculator side_back_buffer_cost_;
  Calculator collision_cost_;

  bool Detects(Vehicle::State state) const;
};


#endif /* lane_h */
