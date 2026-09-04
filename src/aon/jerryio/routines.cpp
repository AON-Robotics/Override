#include "../../../include/aon/jerryio/routines.hpp"

#include "../../../include/aon/constants.hpp"
#include "../../../include/aon/drivetrain/drivetrain.hpp"
#include "../../../include/aon/jerryio/asset.hpp"
#include "../../../include/aon/jerryio/path-jerryio.hpp"
#include "../../../include/aon/tools/logging.hpp"

#include <string>

AON_JERRYIO_ASSET(path_jerryio_txt);

namespace aon::jerryio {
namespace {

const char* motionStatusName(MotionStatus status) {
  switch (status) {
    case MotionStatus::Running:
      return "running";
    case MotionStatus::Completed:
      return "completed";
    case MotionStatus::InvalidPath:
      return "invalid path";
    case MotionStatus::InvalidOptions:
      return "invalid options";
    case MotionStatus::TimedOut:
      return "timed out";
    case MotionStatus::Disabled:
      return "disabled";
    case MotionStatus::Cancelled:
      return "cancelled";
  }
  return "unknown";
}

}  // namespace

int RunPathJerryIOAuton(Drivetrain& drivetrain) {
  const Pose start{0.0, 0.0, 0.0};
  const auto decoded = PathJerryIO::decode(
      reinterpret_cast<const char*>(path_jerryio_txt.data),
      path_jerryio_txt.size);
  if (!decoded) {
    drivetrain.stop();
    logging::Error("PATH.JERRYIO autonomous failed to decode at line " +
                   std::to_string(decoded.line));
    return 0;
  }

  drivetrain.resetPose(start.x, start.y, start.theta);
  FollowPathOptions options;
  options.lookaheadDistance = 8.0;
  options.timeoutMs = 30000;
  options.maximumRpm = 150.0;
  options.finalHeading = 0.0;

  const MotionResult result = drivetrain.followPath(decoded.path, options);
  if (!result) {
    logging::Warn(std::string("PATH.JERRYIO autonomous ") +
                  motionStatusName(result.status));
    return 0;
  }
  logging::Debug("PATH.JERRYIO autonomous completed");
  return 1;
}

}  // namespace aon::jerryio
