#include "../../../include/aon/jerryio/routine-actions.hpp"

#include <utility>

namespace aon::jerryio {

std::vector<PathAction> makePathJerryIOActions(
    std::function<void()> intake, std::function<void()> outtake,
    std::function<void()> stop) {
  return {
      {1, 2000, intake, stop},
      {2, 2000, std::move(outtake), stop},
      {3, 2000, std::move(intake), std::move(stop)},
  };
}

}  // namespace aon::jerryio
