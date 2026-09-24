#include "aon/competition/path-sequence.hpp"
#include "aon/competition/path-tuning.hpp"
#include "aon/generated/static-path.hpp"
#include "aon/tools/path-trace.hpp"
#include "pros/misc.h"
#include "pros/rtos.hpp"
#include "pros/screen.hpp"
#include <functional>

namespace aon {
namespace {
struct TraceLeg {
  PathTrace& trace;
  std::uint32_t started;
  std::uint16_t leg;
  static void sample(void* context, const FollowSample& sample) {
    auto& self = *static_cast<TraceLeg*>(context);
    self.trace.record(sample,pros::millis()-self.started,self.leg);
  }
};
struct MechanismAction {
  const std::function<void(int)>& intake;
  const std::function<void()>& piston;
  int stage;
  static void run(void* context) {
    auto& action = *static_cast<MechanismAction*>(context);
    if (action.stage == 2) action.piston();
    else action.intake(action.stage == 0 ? INTAKE_VELOCITY : -INTAKE_VELOCITY);
  }
  static void stop(void* context) { static_cast<MechanismAction*>(context)->intake(0); }
};
}

int runStaticPath(Drivetrain& drivetrain, const std::function<void(int)>& intake,
                  const std::function<void()>& piston) {
  // Transform once, then borrow overlapping slices. No copies or re-anchoring.
  FollowOptions tuned;
  std::uint32_t profile = 0;
  const auto loaded = loadPathTuning(tuned,profile,"/usd/aon-path-approved.csv");
  if (loaded == TuningLoad::Invalid) {
    intake(0);
    drivetrain.stop();
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,3,"Invalid approved tuning - no motion");
    return 0;
  }
  const auto start = drivetrain.getPose();
  const auto route = generated::staticRouteAt(start,"testing");
  const auto started = pros::millis();
  PathTrace trace(started,TESTING_AUTONOMOUS ? 320 : 0);
  TraceLeg traces[] = {{trace,started,1},{trace,started,2},{trace,started,3}};
  PathStep steps[3];
  MechanismAction actions[] = {{intake,piston,0},{intake,piston,1},{intake,piston,2}};
  std::size_t first = 0;
  const bool valid = route.points.size() >= 2 && route.stops.size() == 3;
  for (std::size_t stage=0; valid && stage<3; ++stage) {
    const auto& stop = route.stops[stage];
    const auto last = stop.index;
    steps[stage].path = route.view().slice(first,last);
    if (loaded == TuningLoad::Loaded) steps[stage].options = tuned;
    steps[stage].options.finalHeading = stop.heading;
    steps[stage].arrived = MechanismAction::run;
    if (stage < 2) steps[stage].afterWait = MechanismAction::stop;
    steps[stage].context = &actions[stage];
    steps[stage].waitMs = stage < 2 ? 2000 : 0;
    if (TESTING_AUTONOMOUS) steps[stage].hooks = {&traces[stage],TraceLeg::sample,nullptr};
    first = last;
  }
  pros::screen::set_eraser(pros::Color::black);
  pros::screen::erase();
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_LARGE_CENTER,1,"JERRYIO TESTING");
  intake(0);
  const auto result = valid ? runPathSequence(drivetrain,steps,3,30000) : Drivetrain::FollowResult::InvalidPath;
  intake(0);
  drivetrain.stop();
  if (TESTING_AUTONOMOUS) {
    Pose target = route.points.empty() ? start : route.points.back();
    if (!route.stops.empty()) target.theta = route.stops.back().heading;
    const bool saved = trace.save("/usd/aon-testing-v2",followResultName(result),drivetrain.getPose(),
        target,pros::millis()-started,&tuned,profile,route.revision);
    pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,6,saved ? "CSV saved to SD" : "CSV not saved (check SD)");
  }
  const char* status = followResultName(result);
  pros::screen::print(pros::E_TEXT_MEDIUM_CENTER,3,"%-32s",status);
  pros::c::controller_print(pros::E_CONTROLLER_MASTER,0,0,"AON: %-15s",status);
  return result == Drivetrain::FollowResult::Completed ? 1 : 0;
}
} // namespace aon
