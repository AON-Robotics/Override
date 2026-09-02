#include "../include/aon/odometry/pose-estimator.hpp"

#include <cmath>

namespace aon::localization {

LocalMotion localMotion(WheelDeltas wheels,
                        TrackingGeometry geometry) noexcept {
  // Calculate change in heading from differential wheel motion
  double deltaTheta = 0.0;
  
  if (wheels.leftValid && wheels.rightValid) {
    const double trackWidth = geometry.leftOffsetInches + geometry.rightOffsetInches;
    if (trackWidth > 0.0) {
      deltaTheta = (wheels.leftInches - wheels.rightInches) / trackWidth;
    }
  }

  // Calculate forward/right motion in robot frame
  double forwardInches = 0.0;
  double rightInches = 0.0;

  // If primarily rotating (small linear motion)
  if (std::abs(deltaTheta) > 1e-4) {
    // Arc motion calculation
    double sign = (deltaTheta > 0) ? 1.0 : -1.0;
    
    if (wheels.leftValid && wheels.rightValid) {
      double radiusLeft = wheels.leftInches / deltaTheta - sign * geometry.leftOffsetInches;
      double radiusRight = wheels.rightInches / deltaTheta + sign * geometry.rightOffsetInches;
      double averageRadius = (radiusLeft + radiusRight) / 2.0;
      
      forwardInches = averageRadius * std::sin(deltaTheta);
      rightInches = averageRadius * (1.0 - std::cos(deltaTheta));
    }
  } 
  // If moving straight or nearly straight
  else {
    // Average left and right encoders for forward motion
    if (wheels.leftValid && wheels.rightValid) {
      forwardInches = (wheels.leftInches + wheels.rightInches) / 2.0;
    } else if (wheels.leftValid) {
      forwardInches = wheels.leftInches;
    } else if (wheels.rightValid) {
      forwardInches = wheels.rightInches;
    }
  }

  // Include back encoder if available and valid for lateral motion
  if (wheels.backValid) {
    // Back encoder primarily measures lateral (right) motion
    // but if we have it, use it for better lateral estimate
    if (wheels.leftValid && wheels.rightValid) {
      // Weight back encoder with forward estimate
      rightInches = (rightInches + wheels.backInches) / 2.0;
    } else {
      rightInches = wheels.backInches;
    }
  }

  return {rightInches, forwardInches, deltaTheta, 
          wheels.leftValid && wheels.rightValid};
}

EstimatorPose propagatePose(EstimatorPose pose,
                            LocalMotion motion) noexcept {
  if (!std::isfinite(motion.forwardInches) || 
      !std::isfinite(motion.rightInches) ||
      !std::isfinite(motion.headingRadians)) {
    return pose;
  }

  // Update heading first
  double newHeading = wrapRadians(pose.headingRadians + motion.headingRadians);

  // Calculate field-frame displacement using average heading (for better accuracy)
  double avgHeading = pose.headingRadians + motion.headingRadians / 2.0;
  double cosHeading = std::cos(avgHeading);
  double sinHeading = std::sin(avgHeading);

  // Transform robot-local motion to field frame
  // Forward motion rotates with the robot's heading
  // Right motion is perpendicular to forward
  double fieldX = motion.forwardInches * cosHeading - 
                  motion.rightInches * sinHeading;
  double fieldY = motion.forwardInches * sinHeading + 
                  motion.rightInches * cosHeading;

  return {pose.xInches + fieldX, pose.yInches + fieldY, newHeading};
}

}  // namespace aon::localization
