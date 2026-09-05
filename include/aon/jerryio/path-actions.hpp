#pragma once

#include "./path.hpp"

#include <cstddef>
#include <cstdint>
#include <functional>
#include <optional>
#include <vector>

namespace aon {

/// Work to perform while stopped at a one-based internal zero-speed marker.
struct PathAction {
  std::size_t markerOrdinal = 0;
  std::uint32_t durationMs = 0;
  std::function<void()> start;
  std::function<void()> cleanup;
};

struct PathLeg {
  Path path;
  std::optional<std::size_t> markerOrdinalAfter;
};

struct PathActionPlan {
  std::vector<PathLeg> legs;
  std::size_t markerCount = 0;
  bool valid = false;
};

/// Splits a path at internal zero-speed points. The final point is never an
/// action marker, even when its speed is zero.
PathActionPlan buildPathActionPlan(const Path& path);

/// Returns false when an action references a marker absent from the plan.
bool validatePathActions(const PathActionPlan& plan,
                         const std::vector<PathAction>& actions);

}  // namespace aon
