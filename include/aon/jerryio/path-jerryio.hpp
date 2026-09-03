#pragma once

#include "./path.hpp"

#include <cstddef>
#include <string_view>

namespace aon {

enum class PathDecodeError {
  None,
  EmptyInput,
  MalformedPoint,
  MissingTerminator,
  TooFewPoints,
  NonFiniteValue,
  SpeedOutOfRange,
};

struct PathDecodeResult {
  Path path;
  PathDecodeError error = PathDecodeError::None;
  std::size_t line = 0;

  explicit operator bool() const { return error == PathDecodeError::None; }
};

/// Decoder for PATH.JERRYIO's LemLib-v0.5 text data section.
/// The format name describes the external grammar; this implementation has no
/// LemLib dependency.
class PathJerryIO {
 public:
  static PathDecodeResult decode(std::string_view input);
  static PathDecodeResult decode(const char* data, std::size_t size);
};

}  // namespace aon
