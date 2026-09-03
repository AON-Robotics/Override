#pragma once

#include <cstddef>
#include <cstdint>

namespace aon {

struct JerryIOAsset {
  const std::uint8_t* data;
  std::size_t size;
};

}  // namespace aon

/// Declares a file from static/ embedded by the AON Makefile rules.
/// Hyphens and periods in the filename are replaced by underscores.
#define AON_JERRYIO_ASSET(name)                                              \
  extern "C" {                                                              \
  extern const std::uint8_t _binary_static_##name##_start[];                 \
  extern const std::uint8_t _binary_static_##name##_end[];                   \
  }                                                                          \
  static const aon::JerryIOAsset name{                                       \
      _binary_static_##name##_start,                                         \
      static_cast<std::size_t>(_binary_static_##name##_end -                 \
                               _binary_static_##name##_start)}
