#include "../../../include/aon/jerryio/path-jerryio.hpp"

#include <cmath>
#include <cstdlib>
#include <string>

namespace aon {
namespace {

void skipWhitespace(const char*& cursor) {
  while (*cursor == ' ' || *cursor == '\t' || *cursor == '\r') ++cursor;
}

bool consumeComma(const char*& cursor) {
  skipWhitespace(cursor);
  if (*cursor != ',') return false;
  ++cursor;
  return true;
}

bool readNumber(const char*& cursor, double& value) {
  skipWhitespace(cursor);
  char* end = nullptr;
  value = std::strtod(cursor, &end);
  if (end == cursor) return false;
  cursor = end;
  return true;
}

PathDecodeResult failure(PathDecodeError error, std::size_t line) {
  return {{}, error, line};
}

PathDecodeError parsePoint(std::string_view text, Path& path) {
  const std::string buffer(text);
  const char* cursor = buffer.c_str();
  PathPoint point;

  if (!readNumber(cursor, point.pose.x) || !consumeComma(cursor) ||
      !readNumber(cursor, point.pose.y) || !consumeComma(cursor) ||
      !readNumber(cursor, point.speed)) {
    return PathDecodeError::MalformedPoint;
  }
  skipWhitespace(cursor);
  if (*cursor != '\0') {
    return PathDecodeError::MalformedPoint;
  }
  if (!std::isfinite(point.pose.x) || !std::isfinite(point.pose.y) ||
      !std::isfinite(point.speed)) {
    return PathDecodeError::NonFiniteValue;
  }
  if (point.speed < 0.0 || point.speed > 127.0) {
    return PathDecodeError::SpeedOutOfRange;
  }

  path.push_back(point);
  return PathDecodeError::None;
}

std::string_view trim(std::string_view text) {
  while (!text.empty() &&
         (text.front() == ' ' || text.front() == '\t' ||
          text.front() == '\r')) {
    text.remove_prefix(1);
  }
  while (!text.empty() &&
         (text.back() == ' ' || text.back() == '\t' ||
          text.back() == '\r')) {
    text.remove_suffix(1);
  }
  return text;
}

}  // namespace

PathDecodeResult PathJerryIO::decode(const char* data, std::size_t size) {
  if (data == nullptr || size == 0) {
    return failure(PathDecodeError::EmptyInput, 0);
  }
  return decode(std::string_view(data, size));
}

PathDecodeResult PathJerryIO::decode(std::string_view input) {
  if (input.empty()) return failure(PathDecodeError::EmptyInput, 0);

  Path path;
  bool foundTerminator = false;
  std::size_t lineNumber = 0;
  std::size_t offset = 0;

  while (offset < input.size()) {
    ++lineNumber;
    const std::size_t newline = input.find('\n', offset);
    const std::size_t end =
        newline == std::string_view::npos ? input.size() : newline;
    const std::string_view line = trim(input.substr(offset, end - offset));

    if (line == "endData") {
      foundTerminator = true;
      break;
    }

    const PathDecodeError error = parsePoint(line, path);
    if (error != PathDecodeError::None) return failure(error, lineNumber);

    if (newline == std::string_view::npos) break;
    offset = newline + 1;
  }

  if (!foundTerminator) {
    return failure(PathDecodeError::MissingTerminator, lineNumber);
  }

  // PATH.JERRYIO 0.11 may append a duplicated zero-speed endpoint and one
  // editor-only zero-speed position before endData. The metadata still names
  // the first of those three points as the real segment endpoint.
  if (path.size() >= 3) {
    const PathPoint& endpoint = path[path.size() - 3];
    const PathPoint& duplicate = path[path.size() - 2];
    const PathPoint& editorPosition = path.back();
    const bool sameEndpoint = endpoint.pose.x == duplicate.pose.x &&
                              endpoint.pose.y == duplicate.pose.y;
    if (sameEndpoint && endpoint.speed == 0.0 && duplicate.speed == 0.0 &&
        editorPosition.speed == 0.0) {
      path.resize(path.size() - 2);
    }
  }

  if (path.size() < 2) {
    return failure(PathDecodeError::TooFewPoints, lineNumber);
  }
  return {std::move(path), PathDecodeError::None, 0};
}

}  // namespace aon
