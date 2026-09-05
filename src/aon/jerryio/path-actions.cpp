#include "../../../include/aon/jerryio/path-actions.hpp"

namespace aon {
namespace {

Path makeLeg(const Path& path, std::size_t first, std::size_t last) {
  Path leg(path.begin() + first, path.begin() + last + 1);
  if (first > 0) leg.front().speed = path[first + 1].speed;
  return leg;
}

}  // namespace

PathActionPlan buildPathActionPlan(const Path& path) {
  PathActionPlan plan;
  if (path.size() < 2 || path.front().speed == 0.0) return plan;

  std::size_t legStart = 0;
  for (std::size_t index = 1; index + 1 < path.size(); ++index) {
    if (path[index].speed != 0.0) continue;
    if (path[index + 1].speed == 0.0) return {};

    ++plan.markerCount;
    plan.legs.push_back(
        {makeLeg(path, legStart, index), plan.markerCount});
    legStart = index;
  }

  plan.legs.push_back({makeLeg(path, legStart, path.size() - 1),
                       std::nullopt});
  plan.valid = true;
  return plan;
}

bool validatePathActions(const PathActionPlan& plan,
                         const std::vector<PathAction>& actions) {
  if (!plan.valid) return false;
  for (const PathAction& action : actions) {
    if (action.markerOrdinal == 0 ||
        action.markerOrdinal > plan.markerCount) {
      return false;
    }
  }
  return true;
}

}  // namespace aon
