#pragma once

#include "./path-actions.hpp"

#include <functional>
#include <vector>

namespace aon::jerryio {

/// Builds the three mechanism actions required by the checked-in route while
/// keeping robot-specific intake hardware outside the JerryIO module.
std::vector<PathAction> makePathJerryIOActions(
    std::function<void()> intake, std::function<void()> outtake,
    std::function<void()> stop);

}  // namespace aon::jerryio
