#include "aon/jerryio/path-transform.hpp"

#include <cmath>
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

constexpr double kTolerance = 0.0001;

bool near(double actual, double expected) {
  return std::abs(actual - expected) <= kTolerance;
}

void rebasesAndRotatesWithoutChangingTheRoute() {
  const aon::Path absolute{{{10, 20, 0}, 100}, {{20, 20, 0}, 0},
                           {{20, 10, 0}, 80}};

  const aon::RelativePath relative = aon::makePathRelative(absolute);

  CHECK(relative.valid);
  CHECK(relative.path.size() == absolute.size());
  CHECK(near(relative.path[0].pose.x, 0.0));
  CHECK(near(relative.path[0].pose.y, 0.0));
  CHECK(near(relative.path[1].pose.x, 10.0));
  CHECK(near(relative.path[1].pose.y, 0.0));
  CHECK(near(relative.path[2].pose.x, 10.0));
  CHECK(near(relative.path[2].pose.y, 10.0));
  CHECK(relative.path[1].speed == 0.0);
  CHECK(relative.path[2].speed == 80.0);
  CHECK(near(relative.path[0].pose.distanceTo(relative.path[1].pose), 10.0));
  CHECK(near(relative.path[1].pose.distanceTo(relative.path[2].pose), 10.0));
  CHECK(near(relative.finalHeading, 90.0));
}

void preservesPhysicalTurnDirectionFromOtherStartHeadings() {
  // North, then east is a right turn in exported field coordinates.
  const auto right = aon::makePathRelative(
      {{{10, 20, 0}, 100}, {{10, 30, 0}, 100}, {{20, 30, 0}, 0}});
  CHECK(right.valid);
  CHECK(near(right.path[1].pose.x, 10.0));
  CHECK(near(right.path[2].pose.y, 10.0));
  CHECK(near(right.finalHeading, 90.0));
  const auto left = aon::makePathRelative(
      {{{10, 20, 0}, 100}, {{10, 30, 0}, 100}, {{0, 30, 0}, 0}});
  CHECK(left.valid);
  CHECK(near(left.path[2].pose.y, -10.0));
  CHECK(near(left.finalHeading, 270.0));
}

void rejectsAPathWithoutAnInitialDirection() {
  const aon::Path path{{{4, 5, 0}, 100}, {{4, 5, 0}, 0}};

  CHECK(!aon::makePathRelative(path).valid);
}

}  // namespace

int main() {
  rebasesAndRotatesWithoutChangingTheRoute();
  preservesPhysicalTurnDirectionFromOtherStartHeadings();
  rejectsAPathWithoutAnInitialDirection();
  std::cout << "AON relative path tests passed\n";
}
