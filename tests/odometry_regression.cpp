// Host regression tests for the production estimator with simulated PROS sensors.
// Run: c++ -std=c++17 -I src tests/odometry_regression.cpp -o /tmp/odometry-test && /tmp/odometry-test
#include <cassert>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <iostream>
#include <limits>
#include <vector>

// Replace only the hardware API; compile the real odometry implementation below.
#define _PROS_API_H_
#define PROS_ERR INT32_MAX
#define TIMEOUT_MAX UINT32_MAX
namespace pros {
std::uint32_t now = 100;
std::uint32_t millis() { return now; }
void delay(int) { throw 1; } // Escape the serial loop in the startup test.
struct Mutex {
  void take(std::uint32_t) {}
  void give() {}
};
struct Rotation {
  std::int32_t value = 0;
  explicit Rotation(int) {}
  std::int32_t get_position() { return value; }
};
struct MotorGroup {
  double value = 0;
  std::vector<double> get_position_all() { return {value}; }
};
struct Gps {
  template <typename... Args> explicit Gps(Args...) {}
};
struct Imu {
  explicit Imu(int) {}
  void reset(bool) {}
  bool is_calibrating() { return false; }
  double get_rotation() { return std::numeric_limits<double>::quiet_NaN(); }
};
struct Task {
  template <typename F> explicit Task(F) {}
};
namespace lcd {
  template <typename... Args> void print(Args...) {}
}
} // namespace pros

#include "../src/aon/odometry/odometry.cpp"

namespace {
std::int32_t wheel(double inches) {
  return std::lround(inches * 36000.0 / (M_PI * TRACKING_WHEEL_DIAMETER));
}
double motor(double inches) {
  return inches * 360.0 / (M_PI * DRIVE_WHEEL_DIAMETER * MOTOR_TO_DRIVE_RATIO);
}
void tick(aon::Odometry& odom) { pros::now += 20; odom.update(); }
void near(double actual, double expected) { assert(std::abs(actual - expected) < 0.001); }
}

int main() {
  {
    aon::Odometry odom(1, 2, 3, 4, 5);
    pros::MotorGroup left, right;
    odom.setDriveMotorGroups(&left, &right);
    tick(odom);
    assert(!odom.hasFreshPose()); // A baseline is not a movement estimate.
    for (int step = 1; step <= 20; ++step) {
      odom.encoderLeft.value = odom.encoderRight.value = wheel(10 * step);
      left.value = right.value = motor(step);
      tick(odom);
      near(odom.getX(), step);
      assert(odom.getPoseSource() == 1);
      assert(odom.hasFreshPose());
    }
    // Both readable sources now jump: neither may keep the pose fresh.
    for (int step = 1; step <= 20; ++step) {
      odom.encoderLeft.value += wheel(10);
      odom.encoderRight.value += wheel(10);
      left.value += motor(10);
      right.value += motor(10);
      tick(odom);
      near(odom.getX(), 20);
      assert(!odom.hasFreshPose());
      assert(odom.getPoseSource() == 0);
    }
    // Rejected samples still rebase so a subsequent normal delta recovers.
    odom.encoderLeft.value += wheel(1);
    odom.encoderRight.value += wheel(1);
    tick(odom);
    near(odom.getX(), 21);
    assert(odom.getPoseSource() == 2);
    assert(odom.hasFreshPose());
    // Accepted zero movement is a usable stationary estimate.
    tick(odom);
    assert(odom.hasFreshPose());
    pros::now += 301;
    assert(!odom.hasFreshPose());
  }
  {
    aon::Odometry odom(1, 2, 3, 4, 5);
    tick(odom);
    // In-place turn with an implausible back-wheel delta must not translate.
    odom.encoderLeft.value = wheel(1);
    odom.encoderRight.value = wheel(-1);
    odom.encoderBack.value = wheel(10);
    tick(odom);
    near(odom.getX(), 0);
    near(odom.getY(), 0);
    assert(std::abs(odom.getDegrees()) > 1);
    // A subsequent accepted lateral delta is used again.
    odom.encoderBack.value += wheel(1);
    tick(odom);
    near(std::hypot(odom.getX(), odom.getY()), 1);
  }
  {
    aon::Odometry odom(1, 2, 3, 4, 5);
    odom.resetCurrent(10, 20, 30);
    odom.SetPosition(40, 50);
    odom.setDegrees(60);
    assert(odom.getPose() == aon::Pose(40, 50, 60));
    assert(!odom.hasFreshPose());
    try { odom.initialize(); } catch (int) {}
    assert(odom.getPose() == aon::Pose(40, 50, 60));
  }
  std::puts("Odometry regression tests passed");
}
