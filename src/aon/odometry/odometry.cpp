#include "../include/aon/odometry/odometry.hpp"

#include <cstdio>
#include <cstdlib>
#include <limits>

namespace aon {
namespace {
constexpr std::size_t kMaxLine = 96;
constexpr std::uint32_t kPoseTimeoutMs = 300;
constexpr double kSparkPositionWeight = 0.5;
constexpr double kMaximumSparkDisagreementInches = 6.0;
constexpr double kMaximumWheelStepInches = 6.0;

bool rotationInches(pros::Rotation& sensor, bool reversed, double diameter,
                    double& inches) {
  const std::int32_t centidegrees = sensor.get_position();
  if (centidegrees == PROS_ERR || !std::isfinite(diameter) || diameter <= 0) return false;
  inches = (reversed ? -1.0 : 1.0) * centidegrees / 100.0 *
           (M_PI * diameter / 360.0);
  return true;
}

bool motorDegrees(pros::MotorGroup* group, double& degrees) {
  if (!group) return false;
  const auto positions = group->get_position_all();
  const auto units = group->get_encoder_units_all();
  if (positions.empty() || positions.size() != units.size()) return false;
  double total = 0.0;
  for (std::size_t i = 0; i < positions.size(); ++i) {
    // A changing subset of motors would change the absolute average baseline.
    // Only explicitly supported degree encoders may supply fallback motion.
    if (!std::isfinite(positions[i]) || units[i] != pros::MotorUnits::degrees)
      return false;
    total += positions[i] / positions.size();
  }
  degrees = total;
  return std::isfinite(degrees);
}

bool parsePose(const char* line, Pose& result) {
  if (!line || line[0] != 'O' || line[1] != ',') return false;
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
    : leftReversed(left < 0), rightReversed(right < 0), backReversed(back < 0),
      headingFusion(V5_IMU_HEADING_SMOOTHING_SECONDS),
      encoderRight(std::abs(right)), encoderLeft(std::abs(left)),
      encoderBack(std::abs(back)),
      gps(gpsPort, GPS_INITIAL_X, GPS_INITIAL_Y, GPS_INITIAL_HEADING,
          GPS_X_OFFSET, GPS_Y_OFFSET)
#if GYRO_ENABLED
      , gyroscope(gyro)
#endif
{ resetInitial(); }

Odometry::Odometry(const Odometry& other)
    : leftReversed(other.leftReversed), rightReversed(other.rightReversed),
      backReversed(other.backReversed),
      headingFusion(V5_IMU_HEADING_SMOOTHING_SECONDS),
      encoderRight(other.encoderRight), encoderLeft(other.encoderLeft),
      encoderBack(other.encoderBack), gps(other.gps)
#if GYRO_ENABLED
      , gyroscope(other.gyroscope)
#endif
{ resetInitial(); }

Pose Odometry::getPose() {
  pose_mutex.take(TIMEOUT_MAX);
  const Pose copy = currentPose;
  pose_mutex.give();
  return copy;
}
double Odometry::getX() { return getPose().x; }
double Odometry::getY() { return getPose().y; }
double Odometry::getDegrees() { return getPose().theta; }
double Odometry::getHeading() {
  const double wrapped = std::fmod(getDegrees(), 360.0);
  return wrapped < 0.0 ? wrapped + 360.0 : wrapped;
}

double Odometry::getTraveledDistance() {
  pose_mutex.take(TIMEOUT_MAX);
  const double distance = traveledDistance;
  pose_mutex.give();
  return distance;
}

OdometryHealth Odometry::getHealth() {
  pose_mutex.take(TIMEOUT_MAX);
  OdometryHealth result = health;
  const auto now = pros::millis();
  if (hasUpdate && now - lastUpdateMs > kPoseTimeoutMs) {
    result.left = result.right = result.back = result.motors = result.imu =
        SensorHealth::Stale;
    result.leftMotors = result.rightMotors = SensorHealth::Stale;
  }
  if (hasPacket && now - lastPacketMs > kPoseTimeoutMs) result.otos = SensorHealth::Stale;
  pose_mutex.give();
  return result;
}

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
  health.motors = health.leftMotors = health.rightMotors = SensorHealth::Baselining;
  pose_mutex.give();
}

void Odometry::resetMotorBaselines() {
  pose_mutex.take(TIMEOUT_MAX);
  hasMotorSample = false;
  health.motors = health.leftMotors = health.rightMotors = SensorHealth::Baselining;
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
  if (!std::isfinite(x) || !std::isfinite(y)) return;
  pose_mutex.take(TIMEOUT_MAX);
  resetCurrentLocked(x, y, currentPose.theta);
  pose_mutex.give();
}
void Odometry::setDegrees(double degrees) {
  if (!std::isfinite(degrees)) return;
  pose_mutex.take(TIMEOUT_MAX);
  resetCurrentLocked(currentPose.x, currentPose.y, degrees);
  pose_mutex.give();
}
void Odometry::setRadians(double radians) { setDegrees(radians * 180.0 / M_PI); }

void Odometry::resetCurrent(double x, double y, double theta) {
  if (!std::isfinite(x) || !std::isfinite(y) || !std::isfinite(theta)) return;
  pose_mutex.take(TIMEOUT_MAX);
  resetCurrentLocked(x, y, theta);
  pose_mutex.give();
}

void Odometry::resetCurrentLocked(double x, double y, double theta) {
  fieldOrigin = Pose(x, y, theta);
  rawOrigin = rawPose;
  originPending = true;
  hasPacket = false;
  currentPose = fieldOrigin;
  headingFusion.reset(theta);
  headingFusion.update(0.0, 0.0, false);
  fallbackHeading = 0.0;
  sparkConsumed = false;
  wasSparkFresh = false;
  hasTrackingSample = false;
  hadBackSample = false;
  hasMotorSample = false;
  poseSource = 0;
  imuFusing = false;
  traveledDistance = 0.0;
  health = {};
  hasUpdate = false;
}

void Odometry::resetInitial() {
  resetCurrent(INITIAL_ODOMETRY_X, INITIAL_ODOMETRY_Y, INITIAL_ODOMETRY_THETA);
}

bool Odometry::acceptPose(const Pose& raw) {
  pose_mutex.take(TIMEOUT_MAX);
  const auto now = pros::millis();
  const double elapsed = hasPacket ? (now - lastPacketMs) / 1000.0 : 0.0;
  const bool recent = hasPacket && now - lastPacketMs <= kPoseTimeoutMs;
  if (!std::isfinite(raw.x) || !std::isfinite(raw.y) || !std::isfinite(raw.theta) ||
      std::abs(raw.x) > 1e6 || std::abs(raw.y) > 1e6 || std::abs(raw.theta) > 1e9 ||
      (recent && (std::hypot(raw.x - rawPose.x, raw.y - rawPose.y) >
                      std::fmax(6.0, 300.0 * elapsed) ||
                  std::abs(std::remainder(raw.theta - rawPose.theta, 360.0)) > 90.0))) {
    health.otos = SensorHealth::Implausible;
    hasPacket = false;
    wasSparkFresh = false;
    pose_mutex.give();
    return false;
  }
  if (!recent) wasSparkFresh = false;
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
  health.otos = SensorHealth::Healthy;
  pose_mutex.give();
  return true;
}

void Odometry::initialize() {
  // Constructors and pose setters establish the field pose before startup.
#if GYRO_ENABLED
  // Keep the robot still during the V5 IMU's approximately two-second calibration.
  gyroscope.reset(true);
#endif
#if GPS_INITIALIZATION_ENABLED
  initializeFromGps();
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
        ingestOtosLine(line);
      }
      length = 0;
      overflow = false;
    } else if (c != '\r' && !overflow) {
      if (length < kMaxLine - 1) line[length++] = static_cast<char>(c);
      else overflow = true;
    }
  }
}

bool Odometry::ingestOtosLine(const char* line) {
  Pose raw;
  if (!parsePose(line, raw)) {
    if (line && line[0] == 'O' && line[1] == ',') {
      pose_mutex.take(TIMEOUT_MAX);
      health.otos = SensorHealth::Invalid;
      pose_mutex.give();
    }
    return false;
  }
  return acceptPose(raw);
}

void Odometry::update() {
  // Serialize reads, baselines, source changes and setters as one transaction.
  pose_mutex.take(TIMEOUT_MAX);
  const std::uint32_t nowMs = pros::millis();
  const bool wasWheelHeading = health.motors == SensorHealth::Healthy ||
      (health.left == SensorHealth::Healthy &&
       health.right == SensorHealth::Healthy);
  const bool gap = hasUpdate && nowMs - lastUpdateMs > kPoseTimeoutMs;
  const double dt = hasUpdate ? (nowMs - lastUpdateMs) / 1000.0 : 0.02;
  if (gap) {
    hasTrackingSample = hadBackSample = hasMotorSample = wasSparkFresh = false;
    headingFusion.update(fallbackHeading, 0.0, false);
  }
  hasUpdate = true;
  lastUpdateMs = nowMs;
  double left = 0, right = 0, back = 0, motorLeft = 0, motorRight = 0;
  const bool leftValid = rotationInches(encoderLeft, leftReversed,
      LEFT_TRACKING_WHEEL_DIAMETER, left);
  const bool rightValid = rotationInches(encoderRight, rightReversed,
      RIGHT_TRACKING_WHEEL_DIAMETER, right);
  const bool backValid = rotationInches(encoderBack, backReversed,
      BACK_TRACKING_WHEEL_DIAMETER, back);
  const bool motorLeftValid = motorDegrees(driveLeft, motorLeft);
  const bool motorRightValid = motorDegrees(driveRight, motorRight);
  const bool motorsValid = motorLeftValid && motorRightValid;
  auto wheelHealth = [](bool valid, bool baseline, double delta) {
    if (!valid) return SensorHealth::Unavailable;
    if (!baseline) return SensorHealth::Baselining;
    return std::abs(delta) <= kMaximumWheelStepInches ?
        SensorHealth::Healthy : SensorHealth::Implausible;
  };
  const double dl = left - previousLeft;
  const double dr = right - previousRight;
  const double db = back - previousBack;
  health.left = wheelHealth(leftValid, hasTrackingSample, dl);
  health.right = wheelHealth(rightValid, hasTrackingSample, dr);
  health.back = wheelHealth(backValid, hadBackSample, db);
  const bool trackingAccepted = health.left == SensorHealth::Healthy &&
                                health.right == SensorHealth::Healthy;
  const bool backAccepted = health.back == SensorHealth::Healthy;
  const auto track = localization::localMotion(
      {dl, dr, db, trackingAccepted, trackingAccepted, backAccepted},
      {DISTANCE_LEFT_TRACKING_WHEEL_CENTER, DISTANCE_RIGHT_TRACKING_WHEEL_CENTER,
       DISTANCE_BACK_TRACKING_WHEEL_CENTER});
  previousLeft = left;
  previousRight = right;
  previousBack = back;
  hasTrackingSample = leftValid && rightValid;
  hadBackSample = backValid;

  const double inchesPerDegree = M_PI * DRIVE_WHEEL_DIAMETER *
                                 MOTOR_TO_DRIVE_RATIO / 360.0;
  const double ml = (motorLeft - previousMotorLeft) * inchesPerDegree;
  const double mr = (motorRight - previousMotorRight) * inchesPerDegree;
  health.leftMotors = wheelHealth(motorLeftValid, hasMotorSample, ml);
  health.rightMotors = wheelHealth(motorRightValid, hasMotorSample, mr);
  health.motors = !motorsValid ? SensorHealth::Unavailable :
      (health.leftMotors == SensorHealth::Implausible ||
       health.rightMotors == SensorHealth::Implausible) ?
          SensorHealth::Implausible : health.leftMotors;
  const bool motorsAccepted = health.motors == SensorHealth::Healthy;
  const auto motor = localization::localMotion(
      {ml, mr, 0, motorsAccepted, motorsAccepted, false},
      {DRIVE_WIDTH / 2.0, DRIVE_WIDTH / 2.0, 0});
  previousMotorLeft = motorLeft;
  previousMotorRight = motorRight;
  hasMotorSample = motorsValid;

  const bool sparkFresh = hasPacket && nowMs - lastPacketMs <= kPoseTimeoutMs;
  if (hasPacket && !sparkFresh) health.otos = SensorHealth::Stale;
  if ((poseSource == 4 && !trackingAccepted) ||
      (poseSource == 3 && trackingAccepted) ||
      (wasWheelHeading && !trackingAccepted && !motorsAccepted))
    wasSparkFresh = false;
  const bool newSpark = sparkFresh && !sparkConsumed;
  const bool sparkRebased = newSpark && !wasSparkFresh;
  double sparkX = 0, sparkY = 0, sparkHeading = 0;
  if (newSpark) {
    const double angle = (fieldOrigin.theta - rawOrigin.theta) * M_PI / 180.0;
    const double dx = rawPose.x - rawOrigin.x, dy = rawPose.y - rawOrigin.y;
    sparkX = fieldOrigin.x + dx * std::cos(angle) - dy * std::sin(angle);
    sparkY = fieldOrigin.y + dx * std::sin(angle) + dy * std::cos(angle);
    if (sparkRebased) {
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

#if GYRO_ENABLED
  const bool calibrating = gyroscope.is_calibrating();
  const double imuRotation = calibrating ? 0.0 : gyroscope.get_rotation();
  const bool imuValid = !calibrating && std::isfinite(imuRotation);
#else
  const double imuRotation = 0.0;
  const bool imuValid = false;
#endif
  const auto motion = trackingAccepted ? track : motor;
  fallbackHeading += trackingAccepted || motorsAccepted ?
      localization::degrees(motion.headingRadians) : sparkHeading;
  const double previousHeading = currentPose.theta;
  const double heading = headingFusion.update(fallbackHeading, imuRotation,
                                              imuValid, dt);
  imuFusing = headingFusion.usingImu();
  health.imu = !imuValid ? SensorHealth::Unavailable :
      headingFusion.rejectedImu() ? SensorHealth::Implausible :
      imuFusing ? SensorHealth::Healthy : SensorHealth::Baselining;

  // Compensate wheel offsets with wheel rotation, not the delayed/smoothed IMU.
  // Fused heading still orients the translation in the established field frame.
  const double middle = (previousHeading + heading) * M_PI / 360.0;
  double distance = 0.0;
  if (trackingAccepted || motorsAccepted) {
    currentPose.x += motion.forwardInches * std::cos(middle) -
                     motion.rightInches * std::sin(middle);
    currentPose.y += motion.forwardInches * std::sin(middle) +
                     motion.rightInches * std::cos(middle);
    distance = std::hypot(motion.forwardInches, motion.rightInches) /
               std::abs(localization::sinc(motion.headingRadians / 2.0));
  }
  if (newSpark && !sparkRebased) {
    const double errorX = sparkX + sparkOffsetX - currentPose.x;
    const double errorY = sparkY + sparkOffsetY - currentPose.y;
    if (std::hypot(errorX, errorY) <= kMaximumSparkDisagreementInches) {
      const double weight = trackingAccepted || motorsAccepted ?
                            kSparkPositionWeight : 1.0;
      currentPose.x += weight * errorX;
      currentPose.y += weight * errorY;
      if (!trackingAccepted && !motorsAccepted) distance = std::hypot(errorX, errorY);
    } else {
      // Apply the same innovation gate even when OTOS is the only source.
      sparkOffsetX -= errorX;
      sparkOffsetY -= errorY;
      health.otos = SensorHealth::Implausible;
    }
  }
  traveledDistance += distance;
  poseSource = trackingAccepted ? (sparkFresh ? 4 : 2) :
               sparkFresh ? 3 : motorsAccepted ? 1 : 0;
  currentPose.theta = heading;
  if (trackingAccepted || motorsAccepted || newSpark) lastEstimateMs = nowMs;
  pose_mutex.give();
}

bool Odometry::readGps(Pose& result) {
  const auto position = gps.get_position();
  const double error = gps.get_error();
  const double heading = gps.get_heading();
  if (!std::isfinite(position.x) || !std::isfinite(position.y) ||
      !std::isfinite(error) || !std::isfinite(heading) || error < 0.0 ||
      error > GPS_MAX_ERROR_METERS || heading < 0.0 || heading >= 360.0 ||
      std::abs(position.x) > GPS_FIELD_HALF_SIZE_METERS ||
      std::abs(position.y) > GPS_FIELD_HALF_SIZE_METERS) {
    health.gps = SensorHealth::Unavailable;
    return false;
  }
  // PROS: +X east, +Y north, meters; this project's +X is heading zero.
  result = Pose(position.y / 0.0254, position.x / 0.0254, heading);
  health.gps = SensorHealth::Healthy;
  return true;
}

bool Odometry::initializeFromGps() {
  pose_mutex.take(TIMEOUT_MAX);
  Pose initial;
  const bool valid = readGps(initial);
  if (valid) {
    resetCurrentLocked(initial.x, initial.y, initial.theta);
    health.gps = SensorHealth::Healthy;
  }
  pose_mutex.give();
  return valid;
}

Vector Odometry::gpsPosition() {
  pose_mutex.take(TIMEOUT_MAX);
  const double invalid = std::numeric_limits<double>::quiet_NaN();
  Pose position(invalid, invalid, 0);
  readGps(position);
  pose_mutex.give();
  return Vector().SetPosition(position.x, position.y);
}
void Odometry::debug() {
  while (true) {
    const Pose p = getPose();
    pros::lcd::print(0, "Pose X %.2f Y %.2f H %.2f", p.x, p.y, p.theta);
    pros::lcd::print(1, "Pose source %d OTOS %d", getPoseSource(), hasFreshOtos());
    pros::lcd::print(2, "V5 IMU heading %s", isImuFusing() ? "on" : "off");
    pros::lcd::print(3, "Distance %.2f in", getTraveledDistance());
    const auto h = getHealth();
    pros::lcd::print(4, "Health L%d R%d B%d M%d", static_cast<int>(h.left),
                     static_cast<int>(h.right), static_cast<int>(h.back),
                     static_cast<int>(h.motors));
    pros::lcd::print(5, "Health IMU%d OTOS%d GPS%d", static_cast<int>(h.imu),
                     static_cast<int>(h.otos), static_cast<int>(h.gps));
    pros::delay(50);
  }
}
}  // namespace aon
