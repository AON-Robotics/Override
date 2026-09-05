#include "../../../include/aon/jerryio/path-transform.hpp"

#include <cmath>

namespace aon {
namespace {

constexpr double kPi = 3.14159265358979323846;

double normalizeHeading(double heading) {
  while (heading >= 360.0) heading -= 360.0;
  while (heading < 0.0) heading += 360.0;
  return heading;
}

}  // namespace

RelativePath makePathRelative(const Path& path) {
  RelativePath result;
  if (path.size() < 2) return result;

  const double initialDx = path[1].pose.x - path[0].pose.x;
  const double initialDy = path[1].pose.y - path[0].pose.y;
  if (!std::isfinite(initialDx) || !std::isfinite(initialDy) ||
      std::hypot(initialDx, initialDy) <= 1e-9) {
    return result;
  }

  const double initialHeading = std::atan2(initialDx, initialDy);
  const double cosine = std::cos(initialHeading);
  const double sine = std::sin(initialHeading);
  result.path.reserve(path.size());
  for (const PathPoint& point : path) {
    const double translatedX = point.pose.x - path.front().pose.x;
    const double translatedY = point.pose.y - path.front().pose.y;
    PathPoint relative = point;
    relative.pose.x = translatedX * cosine - translatedY * sine;
    relative.pose.y = translatedX * sine + translatedY * cosine;
    result.path.push_back(relative);
  }
  result.path.front().pose.x = 0.0;
  result.path.front().pose.y = 0.0;

  const Pose& previous = result.path[result.path.size() - 2].pose;
  const Pose& endpoint = result.path.back().pose;
  const double finalDx = endpoint.x - previous.x;
  const double finalDy = endpoint.y - previous.y;
  if (std::hypot(finalDx, finalDy) <= 1e-9) return {};

  result.finalHeading = normalizeHeading(
      std::atan2(finalDx, finalDy) * 180.0 / kPi);
  result.valid = true;
  return result;
}

}  // namespace aon
