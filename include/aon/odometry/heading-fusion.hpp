#pragma once

#include <cmath>

namespace aon {

// The V5 IMU supplies heading whenever available. OTOS only carries heading
// through an IMU outage; both readings are continuous clockwise degrees.
class HeadingFusion {
 public:
  explicit HeadingFusion(double imuSmoothingSeconds = 0.02)
      : imuSmoothingSeconds_(imuSmoothingSeconds) {}

  void reset(double fieldHeading) {
    fieldHeading_ = fieldHeading;
    heading_ = fieldHeading;
    hasOtos_ = false;
    hasImu_ = false;
    usingImu_ = false;
  }

  double update(double otosHeading, double imuRotation, bool imuValid,
                double elapsedSeconds = 0.02) {
    if (!hasOtos_) {
      previousOtos_ = otosHeading;
      hasOtos_ = true;
      if (imuValid) {
        originImu_ = previousImu_ = imuRotation;
        hasImu_ = true;
      }
      return heading_;
    }

    const double otosDelta = otosHeading - previousOtos_;
    previousOtos_ = otosHeading;
    usingImu_ = false;
    if (!imuValid) {
      heading_ += otosDelta;
      hasImu_ = false;
    } else if (!hasImu_) {
      // Align a newly calibrated or returning IMU without jumping the pose.
      originImu_ = imuRotation - (heading_ - fieldHeading_);
      previousImu_ = imuRotation;
      hasImu_ = true;
    } else {
      const double imuDelta = imuRotation - previousImu_;
      previousImu_ = imuRotation;
      if (std::fabs(imuDelta) > 90.0) {
        // Treat an IMU reset or implausible single-packet jump as invalid.
        heading_ += otosDelta;
        originImu_ = imuRotation - (heading_ - fieldHeading_);
      } else {
        const double imuFieldHeading = fieldHeading_ + imuRotation - originImu_;
        // Smooth uneven IMU steps without letting OTOS drift into heading.
        const double dt = std::fmax(0.0, std::fmin(0.1, elapsedSeconds));
        const double alpha = dt / (imuSmoothingSeconds_ + dt);
        heading_ += alpha * (imuFieldHeading - heading_);
        usingImu_ = true;
      }
    }
    return heading_;
  }

  bool usingImu() const { return usingImu_; }

 private:
  double imuSmoothingSeconds_;
  double fieldHeading_ = 0.0;
  double heading_ = 0.0;
  double previousOtos_ = 0.0;
  double originImu_ = 0.0;
  double previousImu_ = 0.0;
  bool hasOtos_ = false;
  bool hasImu_ = false;
  bool usingImu_ = false;
};

}  // namespace aon
