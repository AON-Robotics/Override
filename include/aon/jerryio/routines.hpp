#pragma once

namespace aon {

class Drivetrain;

namespace jerryio {

/// Follows the checked-in PATH.JERRYIO validation leg using only AON motion.
int RunPathJerryIOValidation(Drivetrain& drivetrain);

}  // namespace jerryio
}  // namespace aon
