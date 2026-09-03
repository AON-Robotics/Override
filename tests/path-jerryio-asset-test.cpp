#include "aon/jerryio/path-jerryio.hpp"

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
  std::ifstream input("static/path-jerryio-validation.jerryio.txt",
                      std::ios::binary);
  CHECK(input.is_open());
  std::ostringstream bytes;
  bytes << input.rdbuf();

  const auto decoded = aon::PathJerryIO::decode(bytes.str());
  CHECK(decoded);
  CHECK(decoded.path.size() == 35);
  CHECK(std::hypot(decoded.path.front().pose.x + 66.557,
                   decoded.path.front().pose.y + 35.024) < 0.001);
  CHECK(std::hypot(decoded.path.back().pose.x + 12.547,
                   decoded.path.back().pose.y + 21.611) < 0.001);
  CHECK(decoded.path.front().speed == 84.0);
  CHECK(decoded.path.back().speed == 0.0);

  for (std::size_t index = 1; index < decoded.path.size(); ++index) {
    CHECK(decoded.path[index - 1].pose.distanceTo(decoded.path[index].pose) <=
          2.1);
    CHECK(decoded.path[index].speed >= 0.0);
    CHECK(decoded.path[index].speed <= 127.0);
  }

  std::cout << "PATH.JERRYIO embedded asset tests passed\n";
}
