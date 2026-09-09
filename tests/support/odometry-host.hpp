#pragma once
// Compile the production odometry implementation with deterministic sensor IO.
// Only the PROS hardware umbrella is replaced; pose/vector/update math is real.
#define _USE_MATH_DEFINES
#define _PROS_API_H_
#include <cmath>
#include <cassert>
#include <iostream>

namespace pros {
struct Mutex {
  bool take(unsigned) { return true; }
  void give() {}
};
struct Rotation {
  explicit Rotation(short) {}
  double position = 0;
  double get_position() const { return position; }
  void set_position(double value) { position = value; }
  void reset() { position = 0; }
};
struct Imu {
  explicit Imu(short) {}
  double heading = 0;
  double get_heading() const { return heading; }
  void tare() { heading = 0; }
};
struct gps_position_s_t { double x = 0, y = 0; };
struct Gps {
  Gps(short, double, double, double, double, double) {}
  gps_position_s_t get_position() { return {}; }
};
inline void delay(unsigned) {}
namespace lcd {
template <typename... Args> void print(int, const char*, Args...) {}
}
}  // namespace pros

#ifdef _MSC_VER
#pragma warning(push)
// Existing Vector/Odometry methods use parameter names matching members.
#pragma warning(disable: 4458)
#endif
#include "../../src/aon/odometry/odometry.cpp"
#ifdef _MSC_VER
#pragma warning(pop)
#endif
