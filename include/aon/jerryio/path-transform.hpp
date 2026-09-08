#pragma once

#include "./path.hpp"

namespace aon {

struct RelativePath {
  Path path;
  double finalHeading = 0.0;
  bool valid = false;
};

/// Moves a path's first point to the origin and rotates its initial direction
/// to AON heading zero (+X), with +Y to the robot's right and clockwise-positive
/// heading. Distances, physical turn direction, speeds, and markers are preserved.
RelativePath makePathRelative(const Path& path);

}  // namespace aon
