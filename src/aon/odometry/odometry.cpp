#include "../include/aon/odometry/odometry.hpp"

#include <cstdio>
#include <cstdlib>

namespace aon {
namespace {
constexpr std::size_t kMaxLine = 96;
constexpr std::uint32_t kPoseTimeoutMs = 300;

bool parsePose(const char* line, Pose& result) {
  if (line[0] != 'O' || line[1] != ',') return false;
  char* end = nullptr;
  const char* cursor = line + 2;
  const double x = std::strtod(cursor, &end);
  if (end == cursor || *end != ',') return false;
  cursor = end + 1;
  const double y = std::strtod(cursor, &end);
  if (end == cursor || *end != ',') return false;
  cursor = end + 1;
  const double heading = std::strtod(cursor, &end);
  if (end == cursor || *end != '\0' || !std::isfinite(x) ||
      !std::isfinite(y) || !std::isfinite(heading)) return false;
  result = Pose(x, y, heading);
  return true;
}
}  // namespace

Odometry::Odometry(short left, short right, short back, short gpsPort, short gyro)
    : conversionFactor(M_PI * TRACKING_WHEEL_DIAMETER / DEGREES_PER_REVOLUTION),
      headingFusion(V5_IMU_HEADING_SMOOTHING_SECONDS),
      encoderLeft(std::abs(left)), encoderRight(std::abs(right)),
      encoderBack(std::abs(back)),
      gps(gpsPort, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING,
          GPS_X_OFFSET, GPS_Y_OFFSET),
      leftReversed(left < 0), rightReversed(right < 0), backReversed(back < 0)
#if GYRO_ENABLED
      , gyroscope(gyro)
#endif
{}

Odometry::Odometry(const Odometry& other)
    : conversionFactor(other.conversionFactor),
      headingFusion(V5_IMU_HEADING_SMOOTHING_SECONDS),
      encoderLeft(other.encoderLeft), encoderRight(other.encoderRight),
      encoderBack(other.encoderBack), gps(other.gps),
      leftReversed(other.leftReversed), rightReversed(other.rightReversed),
      backReversed(other.backReversed)
#if GYRO_ENABLED
      , gyroscope(other.gyroscope)
#endif
{}

Pose Odometry::getPose() {
  pose_mutex.take(TIMEOUT_MAX);
  const Pose copy = currentPose;
  pose_mutex.give();
  return copy;
}
double Odometry::getX() { return getPose().x; }
double Odometry::getY() { return getPose().y; }
double Odometry::getDegrees() { return getPose().theta; }
double Odometry::getRadians() { return getDegrees() * M_PI / 180.0; }
Vector Odometry::getPosition() {
  const Pose p = getPose();
  return Vector().SetPosition(p.x, p.y);
}

bool Odometry::hasFreshPose() {
  pose_mutex.take(TIMEOUT_MAX);
  const bool fresh = hasPacket && pros::millis() - lastPacketMs <= kPoseTimeoutMs;
  pose_mutex.give();
  return fresh;
}

bool Odometry::isImuFusing() {
  pose_mutex.take(TIMEOUT_MAX);
  const bool active = hasPacket && imuFusing &&
                      pros::millis() - lastPacketMs <= kPoseTimeoutMs;
  pose_mutex.give();
  return active;
}

double Odometry::getOtosDegrees() {
  pose_mutex.take(TIMEOUT_MAX);
  const double heading = fieldOrigin.theta + rawPose.theta - rawOrigin.theta;
  pose_mutex.give();
  return heading;
}

void Odometry::SetPosition(double x, double y) {
  const Pose p = getPose();
  resetCurrent(x, y, p.theta);
}
void Odometry::setDegrees(double degrees) {
  const Pose p = getPose();
  resetCurrent(p.x, p.y, degrees);
}
void Odometry::setRadians(double radians) { setDegrees(radians * 180.0 / M_PI); }

void Odometry::resetCurrent(double x, double y, double theta) {
  pose_mutex.take(TIMEOUT_MAX);
  fieldOrigin = Pose(x, y, theta);
  rawOrigin = rawPose;
  originPending = !hasPacket;
  currentPose = fieldOrigin;
  headingFusion.reset(theta);
  if (hasPacket) headingFusion.update(rawPose.theta, 0.0, false);
  imuFusing = false;
  pose_mutex.give();
}

void Odometry::resetInitial() {
  resetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
}

void Odometry::acceptPose(const Pose& raw) {
#if GYRO_ENABLED
  const bool calibrating = gyroscope.is_calibrating();
  const double imuRotation = calibrating ? 0.0 : gyroscope.get_rotation();
  const bool imuValid = !calibrating && std::isfinite(imuRotation);
#else
  const double imuRotation = 0.0;
  const bool imuValid = false;
#endif
  pose_mutex.take(TIMEOUT_MAX);
  Pose continuous = raw;
  if (hasPacket) {
    continuous.theta += 360.0 * std::round((rawPose.theta - raw.theta) / 360.0);
  }
  rawPose = continuous;
  if (originPending) {
    rawOrigin = continuous;
    originPending = false;
  }
  const double angle = (fieldOrigin.theta - rawOrigin.theta) * M_PI / 180.0;
  const double dx = continuous.x - rawOrigin.x;
  const double dy = continuous.y - rawOrigin.y;
  const std::uint32_t nowMs = pros::millis();
  const double elapsedSeconds = hasPacket ? (nowMs - lastPacketMs) / 1000.0 : 0.0;
  const double fusedHeading = headingFusion.update(continuous.theta,
                                                  imuRotation, imuValid,
                                                  elapsedSeconds);
  imuFusing = headingFusion.usingImu();
  currentPose = Pose(fieldOrigin.x + dx * std::cos(angle) - dy * std::sin(angle),
                     fieldOrigin.y + dx * std::sin(angle) + dy * std::cos(angle),
                     fusedHeading);
  lastPacketMs = nowMs;
  hasPacket = true;
  pose_mutex.give();
}

void Odometry::initialize() {
  resetInitial();
#if GYRO_ENABLED
  // Keep the robot still during the V5 IMU's approximately two-second calibration.
  gyroscope.reset(true);
#endif
  char line[kMaxLine] = {};
  std::size_t length = 0;
  bool overflow = false;
  while (true) {
    const int c = std::fgetc(stdin);
    if (c == EOF) {
      std::clearerr(stdin);
      pros::delay(10);
      continue;
    }
    if (c == '\n') {
      if (!overflow) {
        line[length] = '\0';
        Pose raw;
        if (parsePose(line, raw)) acceptPose(raw);
      }
      length = 0;
      overflow = false;
    } else if (c != '\r' && !overflow) {
      if (length < kMaxLine - 1) line[length++] = static_cast<char>(c);
      else overflow = true;
    }
  }
}

void Odometry::update() {}  // Absolute pose is supplied by the Pi.
Vector Odometry::gpsPosition() { return getPosition(); }
void Odometry::debug() {
  while (true) {
    const Pose p = getPose();
    pros::lcd::print(0, "OTOS X %.2f Y %.2f H %.2f", p.x, p.y, p.theta);
    pros::lcd::print(1, "OTOS %s", hasFreshPose() ? "live" : "missing/stale");
    pros::lcd::print(2, "V5 IMU fusion %s", isImuFusing() ? "on" : "off");
    pros::delay(50);
  }
}
}  // namespace aon
