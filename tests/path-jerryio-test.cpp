#include "aon/jerryio/path-jerryio.hpp"

#include <cstdlib>
#include <iostream>
#include <string>

#define CHECK(condition)                                                     \
  do {                                                                       \
    if (!(condition)) {                                                      \
      std::cerr << __FILE__ << ':' << __LINE__ << ": " << #condition       \
                << '\n';                                                     \
      std::exit(1);                                                          \
    }                                                                        \
  } while (false)

namespace {

using aon::PathDecodeError;
using aon::PathJerryIO;

void decodesPointsAndIgnoresEditorMetadata() {
  const std::string input =
      "0, 1.5, 127\r\n"
      "2.25,-3,64.5\r\n"
      "4, 5, 0\r\n"
      "endData\r\n"
      "127\r\n"
      "#PATH.JERRYIO-DATA {not parsed by the robot}\r\n";

  const auto result = PathJerryIO::decode(input);

  CHECK(result);
  CHECK(result.error == PathDecodeError::None);
  CHECK(result.path.size() == 3);
  CHECK(result.path[0].pose.x == 0.0);
  CHECK(result.path[0].pose.y == 1.5);
  CHECK(result.path[0].speed == 127.0);
  CHECK(result.path[1].pose.x == 2.25);
  CHECK(result.path[1].pose.y == -3.0);
  CHECK(result.path[1].speed == 64.5);
  CHECK(result.path[2].speed == 0.0);
}

void acceptsBoundedBuffersWithoutNullTermination() {
  constexpr char input[] = "0,0,50\n1,1,0\nendData\nignored";

  const auto result = PathJerryIO::decode(input, 25);

  CHECK(result);
  CHECK(result.path.size() == 2);
}

void rejectsMalformedRows() {
  const auto result = PathJerryIO::decode("0,0,50\ninvalid\nendData\n");

  CHECK(!result);
  CHECK(result.error == PathDecodeError::MalformedPoint);
  CHECK(result.line == 2);
  CHECK(result.path.empty());
}

void rejectsExtraPointColumns() {
  const auto result = PathJerryIO::decode("0,0,50,12\n1,1,0\nendData\n");

  CHECK(!result);
  CHECK(result.error == PathDecodeError::MalformedPoint);
  CHECK(result.line == 1);
}

void requiresEndData() {
  const auto result = PathJerryIO::decode("0,0,50\n1,1,0\n");

  CHECK(!result);
  CHECK(result.error == PathDecodeError::MissingTerminator);
  CHECK(result.path.empty());
}

void requiresAtLeastTwoPoints() {
  const auto result = PathJerryIO::decode("0,0,0\nendData\n");

  CHECK(!result);
  CHECK(result.error == PathDecodeError::TooFewPoints);
}

void rejectsNonFiniteCoordinates() {
  const auto result = PathJerryIO::decode("nan,0,50\n1,1,0\nendData\n");

  CHECK(!result);
  CHECK(result.error == PathDecodeError::NonFiniteValue);
  CHECK(result.line == 1);
}

void rejectsSpeedsOutsideJerryIoRange() {
  const auto negative = PathJerryIO::decode("0,0,-1\n1,1,0\nendData\n");
  const auto excessive = PathJerryIO::decode("0,0,128\n1,1,0\nendData\n");

  CHECK(!negative);
  CHECK(negative.error == PathDecodeError::SpeedOutOfRange);
  CHECK(!excessive);
  CHECK(excessive.error == PathDecodeError::SpeedOutOfRange);
}

void rejectsEmptyInput() {
  const auto result = PathJerryIO::decode("");

  CHECK(!result);
  CHECK(result.error == PathDecodeError::EmptyInput);
}

}  // namespace

int main() {
  decodesPointsAndIgnoresEditorMetadata();
  acceptsBoundedBuffersWithoutNullTermination();
  rejectsMalformedRows();
  rejectsExtraPointColumns();
  requiresEndData();
  requiresAtLeastTwoPoints();
  rejectsNonFiniteCoordinates();
  rejectsSpeedsOutsideJerryIoRange();
  rejectsEmptyInput();
  std::cout << "PATH.JERRYIO decoder tests passed\n";
}
