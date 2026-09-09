#include "support/odometry-host.hpp"

int main() {
  aon::Odometry odom(1, 2, 3, 4, 5);
  odom.resetCurrent(0, 0, 0);
  odom.gyroscope.heading = 179;
  odom.update();
  const auto before = odom.getPose();
  // Move 0.2 inches while crossing 180 degrees. Equal tracker displacement
  // gives the center distance for the configured symmetric tracker offsets.
  const double centidegrees = 0.2 * 36000 / (M_PI * TRACKING_WHEEL_DIAMETER);
  odom.encoderLeft.position = centidegrees;
  odom.encoderRight.position = centidegrees;
  odom.gyroscope.heading = 181;
  odom.update();
  const auto after = odom.getPose();
  std::cout << "180-degree crossing: traveled=" << before.distanceTo(after)
            << " heading delta=" << after.theta - before.theta << '\n';
  assert(std::abs(after.theta - before.theta - 2.0) < 0.0001);
  assert(before.distanceTo(after) > 0.199);
  assert(before.distanceTo(after) < 0.201);
  assert(std::abs(after.y - before.y) < 0.00001);

  // And cross the boundary in the opposite direction.
  odom.encoderLeft.position += centidegrees;
  odom.encoderRight.position += centidegrees;
  odom.gyroscope.heading = 179;
  odom.update();
  assert(std::abs(odom.getDegrees() - 179) < 0.0001);
  assert(std::abs(odom.getPose().distanceTo(after) - 0.2) < 0.001);

  // Integrate a right arc from its starting heading, not its ending heading.
  odom.resetCurrent(0, 0, 0);
  const double turn = 10 * M_PI / 180;
  const double units = 36000 / (M_PI * TRACKING_WHEEL_DIAMETER);
  odom.encoderLeft.position += (10 + DISTANCE_LEFT_TRACKING_WHEEL_CENTER) * turn * units;
  odom.encoderRight.position += (10 - DISTANCE_RIGHT_TRACKING_WHEEL_CENTER) * turn * units;
  odom.gyroscope.heading = 10;
  odom.update();
  assert(std::abs(odom.getX() - 10 * std::sin(turn)) < 0.00001);
  assert(std::abs(odom.getY() - 10 * (1 - std::cos(turn))) < 0.00001);
  std::cout << "AON odometry tests passed\n";
}
