#include "aon/jerryio/path-actions.hpp"

#include <cstdlib>
#include <iostream>
#include <vector>

#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #condition       \
                << '\n';                                                     \
      std::exit(1);                                                          \
    }                                                                        \
  } while (false)

namespace {

void splitsAtInternalZeroSpeedMarkers() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 0},
                       {{10, 10, 0}, 80}, {{10, 20, 0}, 0}};

  const aon::PathActionPlan plan = aon::buildPathActionPlan(path);

  CHECK(plan.valid);
  CHECK(plan.markerCount == 1);
  CHECK(plan.legs.size() == 2);
  CHECK(plan.legs[0].markerOrdinalAfter.has_value());
  CHECK(*plan.legs[0].markerOrdinalAfter == 1);
  CHECK(!plan.legs[1].markerOrdinalAfter.has_value());
  CHECK(plan.legs[0].path.back().speed == 0.0);
  CHECK(plan.legs[1].path.front().pose.x == 0.0);
  CHECK(plan.legs[1].path.front().pose.y == 10.0);
  CHECK(plan.legs[1].path.front().speed == 80.0);
  CHECK(plan.legs[1].path.back().speed == 0.0);
}

void keepsMarkerOrdinalsStableAcrossSeveralStops() {
  const aon::Path path{{{0, 0, 0}, 100}, {{0, 5, 0}, 0},
                       {{5, 5, 0}, 100}, {{5, 10, 0}, 0},
                       {{10, 10, 0}, 100}, {{10, 15, 0}, 0}};

  const aon::PathActionPlan plan = aon::buildPathActionPlan(path);

  CHECK(plan.valid);
  CHECK(plan.markerCount == 2);
  CHECK(plan.legs.size() == 3);
  CHECK(*plan.legs[0].markerOrdinalAfter == 1);
  CHECK(*plan.legs[1].markerOrdinalAfter == 2);
}

void validatesOnlyActionsThatReferenceExistingMarkers() {
  const aon::Path path{{{0, 0, 0}, 127}, {{0, 10, 0}, 0},
                       {{0, 20, 0}, 127}, {{0, 30, 0}, 0}};
  const aon::PathActionPlan plan = aon::buildPathActionPlan(path);
  const std::vector<aon::PathAction> noActions;
  const std::vector<aon::PathAction> repeatedActions{
      {1, 100, {}, {}}, {1, 200, {}, {}}};
  const std::vector<aon::PathAction> missingMarker{{2, 100, {}, {}}};

  CHECK(aon::validatePathActions(plan, noActions));
  CHECK(aon::validatePathActions(plan, repeatedActions));
  CHECK(!aon::validatePathActions(plan, missingMarker));
}

void rejectsAZeroSpeedStartingPoint() {
  const aon::Path path{{{0, 0, 0}, 0}, {{0, 10, 0}, 127},
                       {{0, 20, 0}, 0}};

  CHECK(!aon::buildPathActionPlan(path).valid);
}

}  // namespace

int main() {
  splitsAtInternalZeroSpeedMarkers();
  keepsMarkerOrdinalsStableAcrossSeveralStops();
  validatesOnlyActionsThatReferenceExistingMarkers();
  rejectsAZeroSpeedStartingPoint();
  std::cout << "AON path action tests passed\n";
}
