#include "../include/aon/odometry/odometry.hpp"

#include <cstdio>
#include <cstdlib>

namespace aon {
namespace {
constexpr std::size_t kMaxLine = 96;
constexpr std::uint32_t kPoseTimeoutMs = 300;
constexpr double kSparkPositionWeight = 0.5;
constexpr double kMaximumSparkDisagreementInches = 6.0;
constexpr double kMaximumWheelStepInches = 6.0;

bool rotationInches(pros::Rotation& sensor, bool reversed, double& inches) {
  const std::int32_t centidegrees = sensor.get_position();
  if (centidegrees == PROS_ERR) return false;
  inches = (reversed ? -1.0 : 1.0) * centidegrees / 100.0 *
           (M_PI * TRACKING_WHEEL_DIAMETER / 360.0);
  return true;
}

bool motorDegrees(pros::MotorGroup* group, double& degrees) {
  if (!group) return false;
  const auto positions = group->get_position_all();
  double total = 0.0;
  int count = 0;
  for (double position : positions) {
    if (std::isfinite(position)) { total += position; ++count; }
  }
  if (count == 0) return false;
  degrees = total / count;
  return true;
}

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
    : headingFusion(V5_IMU_HEADING_SMOOTHING_SECONDS),
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
    : headingFusion(V5_IMU_HEADING_SMOOTHING_SECONDS),
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
  const bool fresh = poseSource != 0 &&
                     pros::millis() - lastEstimateMs <= kPoseTimeoutMs;
  pose_mutex.give();
  return fresh;
}

bool Odometry::hasFreshOtos() {
  pose_mutex.take(TIMEOUT_MAX);
  const bool fresh = hasPacket && pros::millis() - lastPacketMs <= kPoseTimeoutMs;
  pose_mutex.give();
  return fresh;
}

int Odometry::getPoseSource() {
  pose_mutex.take(TIMEOUT_MAX);
  const int source = pros::millis() - lastEstimateMs <= kPoseTimeoutMs ?
                     poseSource : 0;
  pose_mutex.give();
  return source;
}

void Odometry::setDriveMotorGroups(pros::MotorGroup* left,
                                   pros::MotorGroup* right) {
  pose_mutex.take(TIMEOUT_MAX);
  driveLeft = left;
  driveRight = right;
  hasMotorSample = false;
  pose_mutex.give();
}

void Odometry::resetMotorBaselines() {
  pose_mutex.take(TIMEOUT_MAX);
  hasMotorSample = false;
  pose_mutex.give();
}

bool Odometry::isImuFusing() {
  pose_mutex.take(TIMEOUT_MAX);
  const bool active = poseSource != 0 && imuFusing &&
                      pros::millis() - lastEstimateMs <= kPoseTimeoutMs;
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
  fallbackHeading = 0.0;
  sparkConsumed = false;
  wasSparkFresh = false;
  hasTrackingSample = false;
  hadBackSample = false;
  hasMotorSample = false;
  poseSource = 0;
  imuFusing = false;
  pose_mutex.give();
}

void Odometry::resetInitial() {
  resetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
}

void Odometry::acceptPose(const Pose& raw) {
  pose_mutex.take(TIMEOUT_MAX);
  Pose continuous = raw;
  if (hasPacket) {
    continuous.theta += 360.0 * std::round((rawPose.theta - raw.theta) / 360.0);
  }
  rawPose = continuous;
  sparkConsumed = false;
  if (originPending) {
    rawOrigin = continuous;
    originPending = false;
  }
  lastPacketMs = pros::millis();
  hasPacket = true;
  pose_mutex.give();
}

void Odometry::initialize() {
  resetInitial();
#if GYRO_ENABLED
  // Keep the robot still during the V5 IMU's approximately two-second calibration.
  gyroscope.reset(true);
#endif
  pros::Task sensorTask([this] {
    while (true) {
      update();
      pros::delay(20);
    }
  });
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

void Odometry::update() {
  double left = 0, right = 0, back = 0, motorLeft = 0, motorRight = 0;
  const bool leftValid = rotationInches(encoderLeft, leftReversed, left);
  const bool rightValid = rotationInches(encoderRight, rightReversed, right);
  const bool backValid = rotationInches(encoderBack, backReversed, back);
  const bool motorLeftValid = motorDegrees(driveLeft, motorLeft);
  const bool motorRightValid = motorDegrees(driveRight, motorRight);
#if GYRO_ENABLED
  const bool calibrating = gyroscope.is_calibrating();
  const double imuRotation = calibrating ? 0.0 : gyroscope.get_rotation();
  const bool imuValid = !calibrating && std::isfinite(imuRotation);
#else
  const double imuRotation = 0.0;
  const bool imuValid = false;
#endif
  const std::uint32_t nowMs = pros::millis();
  pose_mutex.take(TIMEOUT_MAX);
  const bool trackingValid = leftValid && rightValid;
  const bool motorsValid = motorLeftValid && motorRightValid;
  const bool sparkFresh = hasPacket && nowMs - lastPacketMs <= kPoseTimeoutMs;
  if ((poseSource == 4 && !trackingValid) ||
      (poseSource == 3 && trackingValid)) wasSparkFresh = false;
  const bool newSpark = sparkFresh && !sparkConsumed;
  const bool sparkRebased = newSpark && !wasSparkFresh;
  const bool backMotionValid = backValid && hadBackSample;
  double trackForward = 0, trackRight = 0, trackHeading = 0;
  double motorForward = 0, motorHeading = 0;
  double sparkX = 0, sparkY = 0, sparkHeading = 0;

  if (trackingValid) {
    if (hasTrackingSample) {
      const double dLeft = left - previousLeft;
      const double dRight = right - previousRight;
      if (std::abs(dLeft) <= kMaximumWheelStepInches &&
          std::abs(dRight) <= kMaximumWheelStepInches) {
        trackForward = (dLeft + dRight) / 2.0;
        trackHeading = (dLeft - dRight) /
                       (DISTANCE_LEFT_TRACKING_WHEEL_CENTER +
                        DISTANCE_RIGHT_TRACKING_WHEEL_CENTER) * 180.0 / M_PI;
        if (backMotionValid &&
            std::abs(back - previousBack) <= kMaximumWheelStepInches)
          trackRight = back - previousBack;
      }
    }
    previousLeft = left;
    previousRight = right;
    previousBack = back;
    hadBackSample = backValid;
    hasTrackingSample = true;
  } else {
    hasTrackingSample = false;
    hadBackSample = false;
  }

  if (motorsValid) {
    if (hasMotorSample) {
      const double inchesPerDegree = M_PI * DRIVE_WHEEL_DIAMETER *
                                     MOTOR_TO_DRIVE_RATIO / 360.0;
      const double dLeft = (motorLeft - previousMotorLeft) * inchesPerDegree;
      const double dRight = (motorRight - previousMotorRight) * inchesPerDegree;
      if (std::abs(dLeft) <= kMaximumWheelStepInches &&
          std::abs(dRight) <= kMaximumWheelStepInches) {
        motorForward = (dLeft + dRight) / 2.0;
        motorHeading = (dLeft - dRight) / DRIVE_WIDTH * 180.0 / M_PI;
      }
    }
    previousMotorLeft = motorLeft;
    previousMotorRight = motorRight;
    hasMotorSample = true;
  } else {
    hasMotorSample = false;
  }

  if (newSpark) {
    const double angle = (fieldOrigin.theta - rawOrigin.theta) * M_PI / 180.0;
    const double dx = rawPose.x - rawOrigin.x;
    const double dy = rawPose.y - rawOrigin.y;
    sparkX = fieldOrigin.x + dx * std::cos(angle) - dy * std::sin(angle);
    sparkY = fieldOrigin.y + dx * std::sin(angle) + dy * std::cos(angle);
    if (!wasSparkFresh) {
      sparkOffsetX = currentPose.x - sparkX;
      sparkOffsetY = currentPose.y - sparkY;
    } else {
      sparkHeading = rawPose.theta - previousSparkHeading;
    }
    previousSparkHeading = rawPose.theta;
    sparkConsumed = true;
  }
  if (!sparkFresh) wasSparkFresh = false;
  else if (newSpark) wasSparkFresh = true;

  const double fallbackDelta = trackingValid ? trackHeading :
                               motorsValid ? motorHeading : sparkHeading;
  fallbackHeading += fallbackDelta;
  const double dt = lastEstimateMs ? (nowMs - lastEstimateMs) / 1000.0 : 0.02;
  const double previousHeading = currentPose.theta;
  const double heading = headingFusion.update(fallbackHeading, imuRotation,
                                              imuValid, dt);
  imuFusing = headingFusion.usingImu();

  double driveX = 0, driveY = 0;
  if (trackingValid || motorsValid) {
    const double forward = trackingValid ? trackForward : motorForward;
    const double lateral = trackingValid && backMotionValid ?
        trackRight + (heading - previousHeading) * M_PI / 180.0 *
                     DISTANCE_BACK_TRACKING_WHEEL_CENTER : 0.0;
    const double middleHeading = (previousHeading + heading) * M_PI / 360.0;
    driveX = forward * std::cos(middleHeading) - lateral * std::sin(middleHeading);
    driveY = forward * std::sin(middleHeading) + lateral * std::cos(middleHeading);
  }
  if (trackingValid) {
    currentPose.x += driveX;
    currentPose.y += driveY;
    if (sparkFresh && newSpark && !sparkRebased) {
      const double errorX = sparkX + sparkOffsetX - currentPose.x;
      const double errorY = sparkY + sparkOffsetY - currentPose.y;
      if (std::hypot(errorX, errorY) <= kMaximumSparkDisagreementInches) {
        currentPose.x += kSparkPositionWeight * errorX;
        currentPose.y += kSparkPositionWeight * errorY;
      } else {
        sparkOffsetX -= errorX;
        sparkOffsetY -= errorY;
      }
    }
    poseSource = sparkFresh ? 4 : 2;
  } else if (sparkFresh) {
    if (motorsValid) {
      currentPose.x += driveX;
      currentPose.y += driveY;
    }
    if (newSpark && !sparkRebased) {
      const double correction = motorsValid ? kSparkPositionWeight : 1.0;
      currentPose.x += correction * (sparkX + sparkOffsetX - currentPose.x);
      currentPose.y += correction * (sparkY + sparkOffsetY - currentPose.y);
    }
    poseSource = 3;
  } else if (motorsValid) {
    currentPose.x += driveX;
    currentPose.y += driveY;
    poseSource = 1;
  } else {
    poseSource = 0;
  }
  currentPose.theta = heading;
  if (poseSource != 0) lastEstimateMs = nowMs;
  pose_mutex.give();
}
Vector Odometry::gpsPosition() { return getPosition(); }
void Odometry::debug() {
  while (true) {
    const Pose p = getPose();
    pros::lcd::print(0, "Pose X %.2f Y %.2f H %.2f", p.x, p.y, p.theta);
    pros::lcd::print(1, "Pose source %d OTOS %d", getPoseSource(), hasFreshOtos());
    pros::lcd::print(2, "V5 IMU heading %s", isImuFusing() ? "on" : "off");
    pros::delay(50);
  }
}
}  // namespace aon
