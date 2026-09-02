/**
 * \file logging.hpp
 *
 * \author Alberto I. Cruz Salamán
 * \brief Contains global logger object to report runtime error or to generate
 * warnings inside internal classes
 * 
 * \note Ported to PROS4 - uses printf/fprintf instead of okapi::Logger
 */

#ifndef AON_TOOLS_LOGGING_HPP_
#define AON_TOOLS_LOGGING_HPP_

#include <cstdio>
#include <string>

/// \namespace aon::logging
/// \brief Logging functions for general use and error handling
namespace aon::logging {

/// Configuration of global logger
/// Output at the moment limited to the pros terminal
inline void Initialize() {
  // No initialization needed in PROS4, logging to stdout/stderr is available by default
}

/// Log an error inside the project
/// \param message Message to be added with the error
static inline void Error(const std::string& message) {
  std::fprintf(stderr, "[ERROR] %s\n", message.c_str());
}

/// Log a warning inside the project
/// \param message Message to be added with the warning
static inline void Warn(const std::string& message) {
  std::printf("[WARN] %s\n", message.c_str());
}

/// Log a debug message
/// \param message Message to be added to the debug log
static inline void Debug(const std::string& message) {
  std::printf("[DEBUG] %s\n", message.c_str());
}

static inline void Close() {
  // No cleanup needed in PROS4
}

}  // namespace aon::logging

#endif  // AON_TOOLS_LOGGING_HPP_
