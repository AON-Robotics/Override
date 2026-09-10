#include "aon/paths/static-path.hpp"
#include <cmath>
#include <vector>

namespace aon::paths {
std::vector<Pose> staticPathAt(const Pose& start) {
  // EDIT HERE: paste {x, y} rows in the original JerryIO coordinate system.
  // Inches only; no speed or heading column. Keep each route point once.
  // Builds use this list directly and never overwrite it from the .txt file.
  std::vector<Pose> path{
    {-66.739, -0.674},
    {-64.74, -0.633},
    {-62.74, -0.593},
    {-60.74, -0.552},
    {-58.741, -0.512},
    {-56.741, -0.471},
    {-54.742, -0.43},
    {-52.742, -0.39},
    {-50.742, -0.349},
    {-48.743, -0.308},
    {-46.743, -0.268},
    {-44.744, -0.227},
    {-42.744, -0.186},
    {-40.744, -0.156},
    {-38.745, -0.189},
    {-36.752, -0.34},
    {-34.773, -0.625},
    {-32.826, -1.079},
    {-30.941, -1.742},
    {-29.163, -2.651},
    {-27.565, -3.846},
    {-26.26, -5.353},
    {-25.367, -7.135},
    {-25.011, -9.096},
    {-25.17, -11.084},
    {-25.823, -12.969},
    {-26.866, -14.669},
    {-28.231, -16.127},
    {-29.854, -17.289},
    {-31.661, -18.139},
    {-33.588, -18.661},
    {-35.575, -18.858},
    {-37.57, -18.752},
    {-39.529, -18.357},
    {-41.499, -18.119},
    {-43.498, -18.086},
    {-45.498, -18.053},
    {-47.498, -18.02},
    {-49.498, -17.987},
    {-51.497, -17.955},
    {-53.497, -17.922},
    {-55.497, -17.889},
    {-57.496, -17.856},
    {-59.496, -17.824},
    {-61.496, -17.791},
    {-63.496, -17.758},
    {-65.495, -17.725},
    {-66.429, -17.71},
  };
  if (path.size() < 2) return {};
  for (std::size_t i = 0; i < path.size(); ++i) {
    if (!std::isfinite(path[i].x) || !std::isfinite(path[i].y) ||
        (i > 0 && path[i].x == path[i-1].x && path[i].y == path[i-1].y)) return {};
  }
  // Translate the first point to the robot and align the first segment forward.
  // JerryIO's Y direction is opposite AON's clockwise-positive coordinates.
  constexpr double pi = 3.14159265358979323846;
  const Pose origin = path.front();
  const double angle = std::atan2(path[1].y-origin.y, path[1].x-origin.x);
  const double c = std::cos(angle), s = std::sin(angle);
  const double radians = start.theta*pi/180.0;
  const double startC = std::cos(radians), startS = std::sin(radians);
  for (auto& point : path) {
    const double dx = point.x-origin.x, dy = point.y-origin.y;
    const double x = dx*c + dy*s, y = dx*s - dy*c;
    point.x = start.x + x*startC - y*startS;
    point.y = start.y + x*startS + y*startC;
    point.theta = start.theta;
  }
  const auto& previous = path[path.size()-2];
  path.back().theta = std::atan2(path.back().y-previous.y,
                               path.back().x-previous.x)*180.0/pi;
  return path;
}
}  // namespace aon::paths
