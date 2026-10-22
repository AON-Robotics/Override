#include "aon/odometry/pose-estimator.hpp"

namespace aon::localization {

LocalMotion localMotion(WheelDeltas wheels, TrackingGeometry geometry) noexcept {
  const double width = geometry.leftOffsetInches + geometry.rightOffsetInches;
  if (!wheels.leftValid || !wheels.rightValid ||
      !std::isfinite(wheels.leftInches) || !std::isfinite(wheels.rightInches) ||
      !std::isfinite(width) || width <= 0.0 ||
      !std::isfinite(geometry.leftOffsetInches) ||
      !std::isfinite(geometry.rightOffsetInches) ||
      !std::isfinite(geometry.backOffsetInches)) return {};

  const double turn = (wheels.leftInches - wheels.rightInches) / width;
  // Remove each parallel wheel's rotational travel, including unequal offsets.
  const double forward = (wheels.leftInches - turn * geometry.leftOffsetInches +
                          wheels.rightInches +
                          turn * geometry.rightOffsetInches) / 2.0;
  const bool lateralValid = wheels.backValid && std::isfinite(wheels.backInches);
  const double right = lateralValid ?
      wheels.backInches + turn * geometry.backOffsetInches : 0.0;
  if (!std::isfinite(turn) || !std::isfinite(forward) ||
      !std::isfinite(right)) return {};
  // Midpoint-frame chords for a constant-twist interval, valid for either sign.
  const double scale = sinc(turn / 2.0);
  return {right * scale, forward * scale, turn, lateralValid};
}

EstimatorPose propagatePose(EstimatorPose pose, LocalMotion motion) noexcept {
  if (!isFinite(pose) || !std::isfinite(motion.forwardInches) ||
      !std::isfinite(motion.rightInches) ||
      !std::isfinite(motion.headingRadians)) return pose;
  const double middle = pose.headingRadians + motion.headingRadians / 2.0;
  const EstimatorPose next{
      pose.xInches + motion.forwardInches * std::cos(middle) -
                     motion.rightInches * std::sin(middle),
      pose.yInches + motion.forwardInches * std::sin(middle) +
                     motion.rightInches * std::cos(middle),
      wrapRadians(pose.headingRadians + motion.headingRadians)};
  return isFinite(next) ? next : pose;
}

}  // namespace aon::localization
