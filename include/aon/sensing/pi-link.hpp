#pragma once

#ifndef AON_SENSING_PI_LINK_HPP_
#define AON_SENSING_PI_LINK_HPP_

#include <cstdint>
#include <string>
#include "pros/rtos.hpp"

namespace aon {

/// @brief Reads the RaspberryPi red_tracker's "R,<inches>\n" and "N,0\n"
/// packets from the brain's USB (debug/user serial) port.
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

  /// @brief Thread-safe reading; invalid after 300 ms without a packet.
  Reading latest();

 private:
  std::string buffer_;
  Reading latest_;
  std::uint32_t lastPacketMs_ = 0;
  bool hasPacket_ = false;
  pros::Mutex mutex_;

  /// @brief Parses a single "<color>,<distance>" line (no trailing newline)
  /// @return `true` if `line` was well-formed and `out` was populated
  static bool parseLine(const std::string& line, Reading& out);
};

}  // namespace aon

#endif  // AON_SENSING_PI_LINK_HPP_
