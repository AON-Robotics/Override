#include "aon/competition/path-sequence.hpp"
#include "aon/generated/static-path.hpp"
#include "aon/tools/path-trace.hpp"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"

namespace aon {
int runPathDiagnostic(Drivetrain& drive, int index) {
  const char* names[] = {"diagnostic-straight","diagnostic-curve","path"};
  const char* files[] = {"/usd/aon-straight","/usd/aon-curve","/usd/aon-uturn"};
  if (index < 0 || index > 2) return 0;
  const auto route = generated::staticRouteAt(drive.getPose(),names[index]);
  const auto started = pros::millis();
  PathTrace trace(started);
  FollowOptions options;
  options.maximumRpm = 120;
  FollowHooks hooks;
  hooks.context = &trace;
  hooks.sample = [](void* context, const FollowSample& sample) {
    static_cast<PathTrace*>(context)->record(sample,sample.elapsedMs);
  };
  pros::screen::print(pros::E_TEXT_LARGE_CENTER,1,"PATH DIAGNOSTIC %d",index+1);
  const auto result = drive.follow(route.view(),options,hooks);
  const auto elapsed = pros::millis()-started;
  const bool saved = trace.save(files[index],followResultName(result),drive.getPose(),
                                route.points.empty() ? drive.getPose() : route.points.back(),elapsed);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,3,"%s - %.2fs",followResultName(result),elapsed/1000.0);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,6,saved ? "CSV saved to SD" : "CSV not saved (check SD)");
  return result == Drivetrain::FollowResult::Completed;
}
} // namespace aon
