#pragma once

namespace aon {

class Drivetrain;

namespace jerryio {

/// Follows the checked-in team PATH.JERRYIO autonomous using only AON motion.
int RunPathJerryIOAuton(Drivetrain& drivetrain);

}  // namespace jerryio
}  // namespace aon
