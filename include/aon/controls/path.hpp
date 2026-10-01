#pragma once

#include "../math/pose.hpp"
#include "../constants.hpp"
#include <cstdint>
#include <limits>
#include <vector>

namespace aon {

// Views borrow immutable storage. The owning route must outlive a run.
struct PathView {
  const Pose* points = nullptr;
  std::size_t count = 0;
  const std::uint8_t* speeds = nullptr; // 0..127; null means full speed.

  PathView() = default;
  PathView(const Pose* data, std::size_t size, const std::uint8_t* limits = nullptr)
      : points(data), count(size), speeds(limits) {}
  PathView(const std::vector<Pose>& path) : points(path.data()), count(path.size()) {}
  const Pose& operator[](std::size_t i) const { return points[i]; }
  const Pose& back() const { return points[count-1]; }
  std::size_t size() const { return count; }
  const Pose* data() const { return points; }
  PathView slice(std::size_t first, std::size_t last) const {
    if (!points || first >= last || last >= count) return {};
    return {points+first, last-first+1, speeds ? speeds+first : nullptr};
  }
};

struct PathRoute {
  std::vector<Pose> points;
  std::vector<std::uint8_t> speeds;
  PathView view() const {
    if (points.size() != speeds.size()) return {};
    return {points.data(), points.size(), speeds.data()};
  }
};

struct FollowOptions {
  std::uint32_t timeoutMs = 30000;
  double maximumRpm = 200;
  double lookahead = 6;             // inches at rest
  double lookaheadAtSpeed = 6;      // inches at maximumRpm; equal disables adaptation
  double positionTolerance = 2;    // inches
  double headingTolerance = 2;     // degrees
  double finalHeading = std::numeric_limits<double>::quiet_NaN(); // native absolute; NaN uses endpoint
  double lateralAcceleration = 35; // inches/s^2; zero disables curvature speed limit
  std::uint32_t settleMs = 150;
  double settledRpm = 5;
  // Per-follow multipliers; the drivetrain's shared motion profiles stay unchanged.
  double accelerationScale = 1, decelerationScale = 1;
  double turnAccelerationScale = 1, turnDecelerationScale = 1;
};

inline bool validFollowOptions(const FollowOptions& o) {
  for (double scale : {o.accelerationScale, o.decelerationScale,
                       o.turnAccelerationScale, o.turnDecelerationScale})
    if (!std::isfinite(scale) || scale < 0.1 || scale > 2) return false;
  return o.timeoutMs > 0 && std::isfinite(o.maximumRpm) && o.maximumRpm > 0 && o.maximumRpm <= MAX_RPM &&
      std::isfinite(o.lookahead) && o.lookahead > 0 && std::isfinite(o.lookaheadAtSpeed) && o.lookaheadAtSpeed > 0 &&
      std::isfinite(o.positionTolerance) && o.positionTolerance > 0 &&
      std::isfinite(o.headingTolerance) && o.headingTolerance > 0 && o.headingTolerance <= 180 &&
      (std::isnan(o.finalHeading) || std::isfinite(o.finalHeading)) &&
      std::isfinite(o.lateralAcceleration) && o.lateralAcceleration >= 0 &&
      std::isfinite(o.settledRpm) && o.settledRpm >= 0;
}

struct FollowSample {
  std::uint32_t elapsedMs = 0;
  Pose pose, target;
  double progress = 0, crossTrackError = 0, endpointError = 0;
  double left = 0, right = 0, measuredLeft = 0, measuredRight = 0;
  bool aligning = false;
};

// Called at 10 Hz for logs, every control tick for progress/sensor actions.
// Callbacks must return promptly; false from update cancels the run.
struct FollowHooks {
  void* context = nullptr;
  void (*sample)(void*, const FollowSample&) = nullptr;
  bool (*update)(void*, double) = nullptr;
};

} // namespace aon
