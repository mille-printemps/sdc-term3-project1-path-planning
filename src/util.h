#ifndef util_h
#define util_h

#include <cmath>

namespace util {

  inline double Distance(double x1, double y1, double x2, double y2) {
    return std::sqrt((x2 - x1) * (x2 - x1) + (y2 - y1) * (y2 - y1));
  }

  inline constexpr double Pi() {
    return M_PI;
  }

  inline double Deg2rad(double degree) {
    return degree * Pi() / 180;
  }

  inline double Rad2deg(double radian) {
    return radian * 180 / Pi();
  }

  inline double MilesPerHour2MetersPerSecond(double mph) {
    return mph / 2.236936;
  }

  inline double MetersPerSecond2MilesPerHour(double mps) {
    return mps * 2.236936;
  }
}

#endif /* util_h */
