#include "aon/competition/path-sequence.hpp"
#include "aon/competition/path-tuning.hpp"
#include "aon/generated/static-path.hpp"
#include "aon/tools/path-trace.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace aon {
int runPathDiagnostic(Drivetrain& drive, int index) {
  const char* names[] = {"diagnostic-straight","diagnostic-curve","path"};
  const char* files[] = {"/usd/aon-straight-v2","/usd/aon-curve-v2","/usd/aon-uturn-v2"};
  if (index < 0 || index > 2) return 0;
  const auto route = generated::staticRouteAt(drive.getPose(),names[index]);
  const auto started = pros::millis();
  PathTrace trace(started);
  FollowOptions options;
  options.maximumRpm = 120;
  std::uint32_t profile = 0;
  if (loadPathTuning(options,profile,"/usd/aon-path-tuning.csv") == TuningLoad::Invalid) {
    drive.stop();
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,3,"Invalid tuning file - no motion");
    return 0;
  }
  if (!route.stops.empty()) options.finalHeading = route.stops.back().heading;
  FollowHooks hooks;
  std::pair<PathTrace*,std::uint32_t> recording{&trace,started};
  hooks.context = &recording;
  hooks.sample = [](void* context, const FollowSample& sample) {
    auto& recording = *static_cast<std::pair<PathTrace*,std::uint32_t>*>(context);
    recording.first->record(sample,pros::millis()-recording.second);
  };
  pros::screen::print(pros::E_TEXT_LARGE_CENTER,1,"PATH %d PROFILE %lu",index+1,static_cast<unsigned long>(profile));
  const auto result = drive.follow(route.view(),options,hooks);
  const auto elapsed = pros::millis()-started;
  Pose commandedTarget = route.points.empty() ? drive.getPose() : route.points.back();
  if (!route.stops.empty()) commandedTarget.theta = route.stops.back().heading;
  const bool saved = trace.save(files[index],followResultName(result),drive.getPose(),
                                commandedTarget,elapsed,&options,profile,route.revision);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,3,"%s - %.2fs",followResultName(result),elapsed/1000.0);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,6,saved ? "CSV saved to SD" : "CSV not saved (check SD)");
  return result == Drivetrain::FollowResult::Completed;
}
} // namespace aon
