#include "support/odometry-host.hpp"

void testOdometry() {
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
  // Mirrored right tracker: odometry applies reversal; PROS gets positive ports.
  aon::Odometry mirrored(19, -18, 5, 0, 16);
  assert(!mirrored.encoderRight.reversed);
  mirrored.resetCurrent(0, 0, 0);
  mirrored.encoderLeft.position = 24 * units;
  mirrored.encoderRight.position = -24 * units;
  mirrored.update();
  std::cout << "24-inch forward with reversed right tracker: " << mirrored.getX() << std::endl;
  assert(std::abs(mirrored.getX() - 24) < 0.00001);
  assert(std::abs(mirrored.getY()) < 0.00001);
  // Reset while encoders are nonzero must use the same signed readings.
  mirrored.resetCurrent(0, 0, 0);
  mirrored.update();
  assert(std::abs(mirrored.getX()) < 0.00001);
  // With a stationary IMU, no tracking wheel may change heading.
  const double fixedHeading = mirrored.getDegrees();
  mirrored.encoderLeft.position += units;
  mirrored.update();
  assert(std::abs(mirrored.getDegrees() - fixedHeading) < 0.00001);
  mirrored.encoderRight.position -= units;
  mirrored.update();
  assert(std::abs(mirrored.getDegrees() - fixedHeading) < 0.00001);
  mirrored.encoderBack.position += units;
  mirrored.update();
  assert(std::abs(mirrored.getDegrees() - fixedHeading) < 0.00001);
  mirrored.gyroscope.heading = 15;
  mirrored.update();
  assert(std::abs(mirrored.getDegrees() - fixedHeading - 15) < 0.00001);
  std::cout << "Tracking wheels leave heading unchanged; IMU controls heading\n";
  std::cout << "AON odometry tests passed\n";
}
