#pragma once
#include "../drivetrain/drivetrain.hpp"

namespace aon {
struct PathEvent {
  double distance; // inches from the beginning of this leg; sorted ascending
  void (*fire)(void*);
  void* context = nullptr;
};

struct PathStep {
  PathView path;
  FollowOptions options;
  const PathEvent* events = nullptr;
  std::size_t eventCount = 0;
  void (*arrived)(void*) = nullptr;
  void (*afterWait)(void*) = nullptr;
  void* context = nullptr;
  bool (*ready)(void*) = nullptr; // optional sensor condition after arrival
  std::uint32_t waitMs = 0; // duration, or maximum wait when ready is supplied
  FollowHooks hooks;
};

Drivetrain::FollowResult runPathSequence(Drivetrain&, const PathStep*, std::size_t count,
                                       std::uint32_t timeoutMs);
const char* followResultName(Drivetrain::FollowResult);
} // namespace aon
