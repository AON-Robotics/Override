#pragma once

#include "../math/pose.hpp"

#include <vector>

namespace aon {

/// A sampled PATH.JERRYIO waypoint and its velocity on the editor's 0-127 scale.
struct PathPoint {
  Pose pose;
  double speed = 0.0;
};

using Path = std::vector<PathPoint>;

}  // namespace aon
