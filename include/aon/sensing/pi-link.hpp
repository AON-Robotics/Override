#pragma once

#ifndef AON_SENSING_PI_LINK_HPP_
#define AON_SENSING_PI_LINK_HPP_

#include <string>
#include "pros/rtos.hpp"

namespace aon {

/// @brief Reads "<color>,<distance_in>\n" packets sent by a Raspberry Pi wired
/// to the brain's USB (debug/user serial) port, e.g. "R,14\n" for a red
/// object detected 14 inches away.
class PiLink {
 public:
  enum class Color : char { Red = 'R', Blue = 'B', Green = 'G', Unknown = '?' };

  struct Reading {
    Color color = Color::Unknown;
    double distanceInches = 0.0;
    bool valid = false;
  };

  /// @brief Blocks reading `stdin` one character at a time and parses complete
  /// lines as they arrive; meant to be run in its own `pros::Task` since it
  /// never returns.
  void run();

  /// @brief Thread-safe copy of the most recent successfully parsed reading
  Reading latest();

 private:
  std::string buffer_;
  Reading latest_;
  pros::Mutex mutex_;

  /// @brief Parses a single "<color>,<distance>" line (no trailing newline)
  /// @return `true` if `line` was well-formed and `out` was populated
  static bool parseLine(const std::string& line, Reading& out);
};

}  // namespace aon

#endif  // AON_SENSING_PI_LINK_HPP_
