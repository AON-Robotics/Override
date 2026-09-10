#pragma once
#include "aon/math/pose.hpp"
#include <vector>

namespace aon::generated {
// Route generated from static/path.jerryio.txt, anchored to the robot's live pose.
std::vector<Pose> staticPathAt(const Pose& start);
}  // namespace aon::generated
