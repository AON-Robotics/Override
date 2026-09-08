#include "aon/competition/basic-uturn.hpp"
#include <cassert>
#include <cmath>
#include <iostream>

int main() {
  const auto plan = aon::basicUTurnPlan();
  double x = 0, y = 0, heading = 0;
  constexpr double pi = 3.14159265358979323846;
  const double inchesPerRpmSecond = MOTOR_TO_DRIVE_RATIO * DRIVE_WHEEL_DIAMETER * pi / 60;
  assert(plan.size() == 3);
  for (const auto& leg : plan) {
    assert(leg.durationMs > 0 && leg.durationMs < 10000);
    assert(leg.leftRpm > 0 && leg.leftRpm <= MAX_RPM);
    assert(leg.rightRpm > 0 && leg.rightRpm <= MAX_RPM);
    // Integrate ideal differential-drive kinematics to verify the route shape.
    for (unsigned t = 0; t < leg.durationMs; ++t) {
      const double v = (leg.leftRpm + leg.rightRpm) * inchesPerRpmSecond / 2;
      const double omega = (leg.rightRpm - leg.leftRpm) * inchesPerRpmSecond / DRIVE_WIDTH;
      x += v * std::cos(heading) / 1000;
      y += v * std::sin(heading) / 1000;
      heading += omega / 1000;
    }
  }
  assert(plan[0].leftRpm == plan[0].rightRpm);
  assert(plan[1].leftRpm > plan[1].rightRpm); // right U-turn
  assert(plan[2].leftRpm == plan[2].rightRpm);
  assert(std::abs(x) < 0.1);
  assert(std::abs(y + 17) < 0.1);
  assert(std::abs(heading + pi) < 0.01);
  std::cout << "AON basic U-turn geometry tests passed\n";
}
