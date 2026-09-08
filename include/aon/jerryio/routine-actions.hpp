#pragma once

#include "./path-actions.hpp"

#include <functional>
#include <vector>

namespace aon::jerryio {

/// Builds an optional three-marker intake/outtake/intake action sequence while
/// keeping robot-specific intake hardware outside the JerryIO module.
std::vector<PathAction> makePathJerryIOActions(
    std::function<void()> intake, std::function<void()> outtake,
    std::function<void()> stop);

}  // namespace aon::jerryio
