#include "aon/jerryio/path-jerryio.hpp"
#include "aon/jerryio/path-follower.hpp"

#include <cmath>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <sstream>

#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #condition       \
                << '\n';                                                     \
      std::exit(1);                                                          \
    }                                                                        \
  } while (false)

int main() {
  std::ifstream input("static/path.jerryio.txt", std::ios::binary);
  CHECK(input.is_open());
  std::ostringstream bytes;
  bytes << input.rdbuf();

  const auto decoded = aon::PathJerryIO::decode(bytes.str());
  CHECK(decoded);
  CHECK(decoded.path.size() == 87);
  CHECK(std::hypot(decoded.path.front().pose.x,
                   decoded.path.front().pose.y) < 0.001);
  CHECK(std::hypot(decoded.path.back().pose.x + 19.887,
                   decoded.path.back().pose.y + 5.225) < 0.001);
  CHECK(decoded.path.front().speed == 20.0);
  CHECK(decoded.path.back().speed == 0.0);

  for (std::size_t index = 1; index < decoded.path.size(); ++index) {
    CHECK(decoded.path[index - 1].pose.distanceTo(decoded.path[index].pose) <=
          2.1);
    CHECK(decoded.path[index - 1].pose.distanceTo(decoded.path[index].pose) >
          0.0);
    CHECK(decoded.path[index].speed >= 0.0);
    CHECK(decoded.path[index].speed <= 127.0);
  }


  aon::PathFollowerConfig config;
  config.maximumAcceleration = 100000.0;
  config.maximumDeceleration = 100000.0;
  const aon::PathFollower followerTemplate(decoded.path, config);
  CHECK(followerTemplate.isValid());
  auto follower = followerTemplate;
  const auto firstCommand = follower.step({0, 0, 0}, 0.02);
  CHECK(firstCommand.valid);
  CHECK(std::abs(firstCommand.leftRpm - firstCommand.rightRpm) > 1.0);

  std::cout << "PATH.JERRYIO embedded asset tests passed\n";
}
