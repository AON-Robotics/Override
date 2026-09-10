#pragma once
#include "aon/math/pose.hpp"
#include <vector>

namespace aon::paths {
// Manually entered route from src/aon/paths/static-path.cpp, anchored to the robot's live pose.
std::vector<Pose> staticPathAt(const Pose& start);
}  // namespace aon::paths
