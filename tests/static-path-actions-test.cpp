#include "support/drivetrain-host.hpp"

void testActions() {
  using Result = aon::Drivetrain::FollowResult;
  {
    pros::reset();
    SimDrive drive;
    pros::advance = [&](unsigned ms) { drive.advance(ms); };
    const std::vector<aon::Pose> points{{0,0,0},{10,0,0}};
    int events = 0;
    aon::PathEvent event{5, [](void* context) { ++*static_cast<int*>(context); }, &events};
    aon::PathStep step;
    step.path = points;
    step.events = &event;
    step.eventCount = 1;
    step.waitMs = 100;
    step.ready = [](void*) { return false; };
    assert(aon::runPathSequence(drive, &step, 1, 30000) == Result::TimedOut);
    assert(events == 1);
    event.distance = -1;
    events = 0;
    assert(aon::runPathSequence(drive, &step, 1, 30000) == Result::InvalidOptions);
    assert(events == 0);
    event.distance = 5;
    aon::PathStep invalidSteps[] = {step,step};
    invalidSteps[1].options.lookahead = 0;
    drive.driveCommands = 0;
    assert(aon::runPathSequence(drive,invalidSteps,2,30000) == Result::InvalidOptions);
    assert(drive.driveCommands == 0);
    pros::advance = nullptr;
  }
  // Two near-endpoint legs isolate waits and the shared deadline without
  // replacing the production follow loop or sequence state machine.
  for (int scenario = 0; scenario < 4; ++scenario) {
    pros::reset();
    SimDrive drive;
    const std::vector<aon::Pose> points{{0,0,0},{1,0,0}};
    struct State { std::vector<int> order; unsigned arrival = 0; } state;
    aon::PathStep steps[2];
    for (auto& step : steps) {
      step.path = points;
      step.context = &state;
      step.arrived = [](void* ctx) {
        auto& state = *static_cast<State*>(ctx);
        state.order.push_back(1);
        state.arrival = pros::millis();
      };
      step.afterWait = [](void* ctx) { static_cast<State*>(ctx)->order.push_back(2); };
      step.waitMs = 100;
    }
    if (scenario == 0) {
      steps[0].ready = [](void* ctx) {
        return pros::millis()-static_cast<State*>(ctx)->arrival >= 30;
      };
    }
    if (scenario == 1) pros::disableAt = 480; // first leg arrives at 450 ms
    if (scenario == 2) pros::cancelAt = 480;
    const auto result = aon::runPathSequence(drive,steps,2,scenario == 3 ? 1020 : 30000);
    if (scenario == 0) {
      assert(result == Result::Completed);
      assert((state.order == std::vector<int>{1,2,1,2}));
      assert(pros::millis() == 1030); // ready ends first wait at 30 ms
    } else if (scenario == 3) {
      assert(result == Result::TimedOut);
      assert((state.order == std::vector<int>{1,2,1,2}));
      assert(pros::millis() == 1020); // second wait uses remaining shared budget
    } else {
      assert(result == (scenario == 1 ? Result::Disabled : Result::Cancelled));
      assert((state.order == std::vector<int>{1,2}));
      assert(pros::millis() == 480);
    }
    assert(drive.targetLeft == 0 && drive.targetRight == 0);
  }
  for (int scenario = 0; scenario < 6; ++scenario) {
    pros::reset();
    SimDrive drive({40,-20,137});
    pros::advance = [&](unsigned ms) { drive.advance(ms); };
    if (scenario == 1) drive.stalled = true;
    if (scenario == 2) pros::cancelAt = 1000;
    if (scenario == 3) pros::disableAt = 1000;
    std::vector<std::pair<int,unsigned>> commands;
    int piston = 0;
    const auto route = aon::generated::staticRouteAt(drive.getPose(),"testing");
    std::size_t arrived = 0;
    auto checkArrival = [&] {
      assert(arrived < route.stops.size());
      const auto stop = route.stops[arrived++];
      assert(drive.getPose().distanceTo(route.points[stop.index]) <= 2.1);
      assert(std::abs(std::remainder(drive.getTheta()-stop.heading,360)) <= 2.01);
      assert(std::abs(drive.actualLeft) <= 5 && std::abs(drive.actualRight) <= 5);
    };
    const int result = aon::runStaticPath(drive,
        [&](int rpm) {
          if (rpm) checkArrival();
          if (rpm > 0 && scenario == 4) pros::cancelAt = pros::millis()+1000;
          if (rpm == 0 && arrived == 1 && scenario == 5) drive.stalled = true;
          commands.push_back({rpm,pros::millis()});
        }, [&] { checkArrival(); ++piston; });
    assert(commands.back().first == 0);
    assert(drive.actualLeft == 0 && drive.actualRight == 0);
    if (scenario == 0) {
      assert(result == 1 && piston == 1 && arrived == 3);
      assert(commands.size() == 6);
      assert(commands[1].first == INTAKE_VELOCITY && commands[2].second-commands[1].second == 2000);
      assert(commands[3].first == -INTAKE_VELOCITY && commands[4].second-commands[3].second == 2000);
    } else {
      assert(result == 0 && piston == 0 && arrived == (scenario >= 4 ? 1u : 0u));
      for (auto command : commands) assert(command.first >= 0);
    }
    pros::advance = nullptr;
  }
  std::cout << "Testing route actions, cancellation and timeout passed\n";
}
