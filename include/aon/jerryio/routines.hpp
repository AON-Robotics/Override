#pragma once

#include "./path-actions.hpp"

#include <vector>

namespace aon {

class Drivetrain;

namespace jerryio {

/// Follows the checked-in team PATH.JERRYIO autonomous using only AON motion.
int RunPathJerryIOAuton(Drivetrain& drivetrain,
                        const std::vector<PathAction>& actions);

}  // namespace jerryio
}  // namespace aon
