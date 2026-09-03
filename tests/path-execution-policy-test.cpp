#include "aon/jerryio/path-following.hpp"

#include <cstdlib>
#include <iostream>

#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #condition       \
                << '\n';                                                     \
      std::exit(1);                                                          \
    }                                                                        \
  } while (false)

namespace {

using aon::FollowPathOptions;
using aon::MotionLoopSnapshot;
using aon::MotionStatus;

void validatesEverySafetyCriticalOption() {
  FollowPathOptions valid;
  CHECK(valid.isValid());

  FollowPathOptions invalid = valid;
  invalid.lookaheadDistance = 0;
  CHECK(!invalid.isValid());
  invalid = valid;
  invalid.timeoutMs = 0;
  CHECK(!invalid.isValid());
  invalid = valid;
  invalid.loopPeriodMs = 0;
  CHECK(!invalid.isValid());
  invalid = valid;
  invalid.maximumRpm = -1;
  CHECK(!invalid.isValid());
  invalid = valid;
  invalid.finalHeading = 400.0;
  CHECK(!invalid.isValid());
}

void safetyStopsTakePriorityOverCompletion() {
  MotionLoopSnapshot state;
  state.pathValid = true;
  state.optionsValid = true;
  state.complete = true;
  state.disabled = true;
  CHECK(aon::evaluateMotionStatus(state) == MotionStatus::Disabled);

  state.disabled = false;
  state.cancelled = true;
  CHECK(aon::evaluateMotionStatus(state) == MotionStatus::Cancelled);
}

void classifiesInvalidRunningCompletedAndTimedOutStates() {
  MotionLoopSnapshot state;
  CHECK(aon::evaluateMotionStatus(state) == MotionStatus::InvalidPath);

  state.pathValid = true;
  CHECK(aon::evaluateMotionStatus(state) == MotionStatus::InvalidOptions);

  state.optionsValid = true;
  CHECK(aon::evaluateMotionStatus(state) == MotionStatus::Running);

  state.complete = true;
  CHECK(aon::evaluateMotionStatus(state) == MotionStatus::Completed);

  state.complete = false;
  state.elapsedMs = 5000;
  state.timeoutMs = 5000;
  CHECK(aon::evaluateMotionStatus(state) == MotionStatus::TimedOut);
}

void onlySuccessfulMotionMayAlignFinalHeading() {
  FollowPathOptions options;
  CHECK(!aon::shouldAlignFinalHeading(MotionStatus::Completed, options));

  options.finalHeading = 90.0;
  CHECK(aon::shouldAlignFinalHeading(MotionStatus::Completed, options));
  CHECK(!aon::shouldAlignFinalHeading(MotionStatus::TimedOut, options));
  CHECK(!aon::shouldAlignFinalHeading(MotionStatus::Disabled, options));
  CHECK(!aon::shouldAlignFinalHeading(MotionStatus::Cancelled, options));
}

void motionResultReportsSuccessOnlyForCompletion() {
  CHECK(aon::MotionResult{MotionStatus::Completed});
  CHECK(!aon::MotionResult{MotionStatus::TimedOut});
}

}  // namespace

int main() {
  validatesEverySafetyCriticalOption();
  safetyStopsTakePriorityOverCompletion();
  classifiesInvalidRunningCompletedAndTimedOutStates();
  onlySuccessfulMotionMayAlignFinalHeading();
  motionResultReportsSuccessOnlyForCompletion();
  std::cout << "AON path execution policy tests passed\n";
}
