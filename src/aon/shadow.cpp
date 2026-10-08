#include "../../include/aon/shadow.hpp"

#include <algorithm>
#include <atomic>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <limits>

namespace aon::shadow {
namespace {

constexpr std::uint8_t kPinned = 1;
constexpr std::uint8_t kPrefix = 1;
constexpr std::uint8_t kVersion = 1;
static_assert(sizeof(float) == 4 && std::numeric_limits<float>::is_iec559);
static ShadowRecording recording; // ONLY full recording buffer; never copied
static bool capturing = false;
static bool ready = false;
static Result result = Result::Empty;
struct CaptureState {
  ShadowPoint previous;
  std::uint32_t start, lastPoll, keyframe;
  float reductionError;
  std::int8_t lastValue[static_cast<unsigned>(Kind::Count)];
  std::int8_t direction;
};
static CaptureState captureState;

enum class UiAction : std::uint8_t {
  None, Slot1, Slot2, Slot3, Record, Save, Load, Delete, Arm, Back, Cancel, Native, Open
};
UiAction touchAction(int x, int y) {
  if (x < 0 || x >= 480 || y < 0 || y >= 240) return UiAction::None;
  if (y >= 45 && y < 78) return static_cast<UiAction>(1 + x / 160);
  if (y >= 140 && y < 178) return static_cast<UiAction>(4 + x / 160);
  if (y >= 185 && y < 225) return static_cast<UiAction>(7 + x / 160);
  if (y >= 5 && y < 35 && x >= 385) return UiAction::Cancel;
  if (y >= 5 && y < 35 && x >= 180 && x < 300) return UiAction::Native;
  return UiAction::None;
}
constexpr std::uint32_t kUiEpochMask = 0x00ffffffU;
bool armCurrent(std::uint32_t authorized, std::uint32_t latest, std::uint32_t pending) {
  return authorized == (latest & kUiEpochMask) && pending == 0;
}

float angle(float a, float b) { return std::remainder(a - b, 360.0F); }
float distance(const ShadowPoint& a, const ShadowPoint& b) {
  return std::hypot(a.x - b.x, a.y - b.y);
}
bool finite(const ShadowPoint& p) {
  return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.heading) &&
         std::fabs(p.x) <= 1000 && std::fabs(p.y) <= 1000 && std::fabs(p.heading) <= 360;
}
bool validEvent(Kind kind, std::int8_t value, std::uint8_t robot) {
  if (kind >= Kind::Count || value < 0) return false;
  if (kind == Kind::Intake) {
    if (value > static_cast<int>(IntakeMode::Middle)) return false;
    return robot == 2 ? value != 2 && value != 3 : value != 6;
  }
  if (kind == Kind::Sort) return robot == 2 && value <= 2;
  if (robot == 2 && (kind == Kind::ScorerHeight || kind == Kind::Trapdoor ||
                    kind == Kind::Lever || kind == Kind::Arrow)) return false;
  if (robot == 1 && kind == Kind::Sem) return false;
  return value <= 1;
}
Result validate(std::uint8_t robot) {
  if (recording.robot != robot || (robot != 1 && robot != 2)) return Result::WrongRobot;
  if (!recording.pointCount || recording.pointCount > kMaxPoints ||
      recording.eventCount > kMaxEvents || recording.durationMs > kMaxDurationMs ||
      recording.flags > kPrefix) return Result::Corrupt;
  bool meaningful = recording.eventCount != 0;
  std::uint32_t dwell = 0;
  for (unsigned i = 0; i < recording.pointCount; ++i) {
    const auto& p = recording.points[i];
    if (!finite(p) || p.direction < -1 || p.direction > 1 || p.flags > kPinned)
      return Result::Corrupt;
    dwell += p.dwellMs;
    if (dwell > recording.durationMs) return Result::Corrupt;
    if (i && (distance(p, recording.points[0]) > 0.15F ||
              std::fabs(angle(p.heading, recording.points[0].heading)) > 1)) meaningful = true;
  }
  std::uint16_t lastIndex = 0, lastDelay = 0;
  for (unsigned i = 0; i < recording.eventCount; ++i) {
    const auto& e = recording.events[i];
    if (e.pointIndex >= recording.pointCount || e.pointIndex < lastIndex ||
        (e.pointIndex == lastIndex && e.delayMs < lastDelay) ||
        e.delayMs > recording.points[e.pointIndex].dwellMs ||
        !validEvent(e.kind, e.value, robot)) return Result::Corrupt;
    lastIndex = e.pointIndex;
    lastDelay = e.delayMs;
  }
  return meaningful ? Result::Ok : Result::Empty;
}
Result abortCapture(Result why) {
  capturing = false;
  ready = false;
  return result = why;
}
Result limitCapture(Result why) {
  capturing = false;
  recording.flags = kPrefix;
  ready = validate(recording.robot) == Result::Ok;
  return result = why;
}
Result startCapture(ShadowPoint p, std::uint32_t now, std::uint8_t robot) {
  if (!finite(p)) return abortCapture(Result::InvalidPose);
  recording.pointCount = 1;
  recording.eventCount = recording.durationMs = recording.flags = 0;
  recording.robot = robot;
  p.dwellMs = 0; p.flags = kPinned;
  recording.points[0] = p;
  captureState = {};
  captureState.previous = p;
  captureState.direction = p.direction;
  captureState.start = captureState.lastPoll = captureState.keyframe = now;
  std::fill_n(captureState.lastValue, static_cast<unsigned>(Kind::Count), static_cast<std::int8_t>(-1));
  capturing = true;
  ready = false;
  return result = Result::Ok;
}
Result append(ShadowPoint p) {
  if (recording.pointCount == kMaxPoints) return limitCapture(Result::Capacity);
  recording.points[recording.pointCount++] = p;
  captureState.reductionError = 0;
  return Result::Ok;
}
// Constant-work online reduction. Only the unpinned tail may be replaced;
// start, corners, direction boundaries, keyframes, dwells and events are pinned.
Result capture(ShadowPoint p, std::uint32_t now, std::int8_t direction, bool force = false,
               bool stationaryAllowed = true) {
  if (!capturing) return result;
  const auto elapsed = now - captureState.start;
  if (elapsed > kMaxDurationMs) return limitCapture(Result::Duration);
  const auto dt = now - captureState.lastPoll;
  if (!force && dt < kPollMs) return Result::Ok;
  if (!finite(p) || dt > 200) return abortCapture(Result::InvalidPose);
  if (distance(p, captureState.previous) > 8 ||
      std::fabs(angle(p.heading, captureState.previous.heading)) > 45)
    return abortCapture(Result::PoseJump);
  p.dwellMs = 0; p.flags = 0; p.direction = direction;
  recording.durationMs = static_cast<std::uint16_t>(elapsed);
  captureState.previous = p;
  captureState.lastPoll = now;
  auto& last = recording.points[recording.pointCount - 1];
  const float travel = distance(last, p);
  const float rotation = std::fabs(angle(last.heading, p.heading));
  const bool changed = direction != captureState.direction;
  const bool stationary = stationaryAllowed && direction == 0 && travel <= 0.15F && rotation <= 1;
  if (stationary) {
    last.flags = kPinned;
    last.direction = 0;
    last.dwellMs = static_cast<std::uint16_t>(last.dwellMs + dt);
    captureState.direction = 0;
    if (elapsed == kMaxDurationMs) return limitCapture(Result::Duration);
    return Result::Ok;
  }
  if (changed) {
    last.flags = kPinned;
    // Direction describes the OUTGOING leg. At a reversal the robot may have
    // already moved between polls; use that leg's forward projection rather
    // than retaining the previous command on the boundary point.
    constexpr float radians = 0.01745329252F;
    const float projection = (p.x - last.x) * std::cos(last.heading * radians) +
                             (p.y - last.y) * std::sin(last.heading * radians);
    if (std::fabs(projection) > 0.02F) last.direction = projection > 0 ? 1 : -1;
    if (last.direction == 0 && direction != 0) last.direction = direction;
    if (travel > 0.02F || rotation > 0.1F) {
      p.flags = kPinned;
      if (append(p) != Result::Ok) return result;
    } else {
      last.direction = direction;
    }
    captureState.direction = direction;
    captureState.keyframe = now;
    return elapsed == kMaxDurationMs ? limitCapture(Result::Duration) : Result::Ok;
  }
  const bool keyframe = now - captureState.keyframe >= kKeyframeMs;
  if (!force && travel < kPositionInches && rotation < kHeadingDegrees && !keyframe)
    return Result::Ok;
  if (force && travel <= 0.02F && rotation <= 0.1F) {
    last.flags = kPinned;
    return Result::Ok;
  }
  p.flags = (force || keyframe || rotation >= kHeadingDegrees) ? kPinned : 0;
  if (recording.pointCount > 1 && last.flags == 0 && last.dwellMs == 0 && !force) {
    const auto& before = recording.points[recording.pointCount - 2];
    const float dx = p.x - before.x, dy = p.y - before.y;
    const float length = std::hypot(dx, dy);
    const float error = length > 0 ?
        std::fabs(dx * (last.y - before.y) - dy * (last.x - before.x)) / length : 1;
    const float dot = (last.x - before.x) * (p.x - last.x) +
                      (last.y - before.y) * (p.y - last.y);
    if (before.direction == direction && dot >= 0 &&
        std::fabs(angle(before.heading, p.heading)) < kHeadingDegrees &&
        captureState.reductionError + error <= 0.15F) {
      last = p;
      captureState.reductionError += error;
    } else {
      last.flags = kPinned;
      if (append(p) != Result::Ok) return result;
    }
  } else if (append(p) != Result::Ok) return result;
  if (p.flags) captureState.keyframe = now;
  return elapsed == kMaxDurationMs ? limitCapture(Result::Duration) : Result::Ok;
}
Result addEvent(Kind kind, std::int8_t value, ShadowPoint p, std::uint32_t now,
                std::int8_t direction, bool stationaryAllowed = true) {
  if (!capturing) return result;
  if (!validEvent(kind, value, recording.robot)) return abortCapture(Result::Corrupt);
  auto& previous = captureState.lastValue[static_cast<unsigned>(kind)];
  if (previous == value) return Result::Ok;
  if (recording.eventCount == kMaxEvents) return limitCapture(Result::Capacity);
  if (capture(p, now, direction, true, stationaryAllowed) != Result::Ok) return result;
  const auto index = static_cast<std::uint16_t>(recording.pointCount - 1);
  recording.points[index].flags = kPinned;
  recording.events[recording.eventCount++] = {index, recording.points[index].dwellMs, kind, value};
  previous = value;
  return Result::Ok;
}
Result finishCapture(ShadowPoint p, std::uint32_t now, std::int8_t direction, bool stationaryAllowed = true) {
  if (capturing) capture(p, now, direction, true, stationaryAllowed);
  if (result != Result::Ok && result != Result::Duration && result != Result::Capacity) return result;
  capturing = false;
  const auto check = validate(recording.robot);
  ready = check == Result::Ok;
  return result = check;
}

// Streaming codec, bounded stack and no raw-struct/padding dependence.
struct Stream {
  FILE* file;
  bool writing;
  bool ok = true;
  std::uint32_t hash = 2166136261U;
  std::uint32_t integer(std::uint32_t value, unsigned bytes, bool checksum = true) {
    std::uint32_t decoded = 0;
    for (unsigned i = 0; i < bytes; ++i) {
      int byte;
      if (writing) {
        byte = static_cast<int>((value >> (8 * i)) & 255);
        if (std::fputc(byte, file) == EOF) ok = false;
      } else {
        byte = std::fgetc(file);
        if (byte == EOF) { ok = false; byte = 0; }
      }
      if (checksum) hash = (hash ^ static_cast<std::uint32_t>(byte)) * 16777619U;
      decoded |= static_cast<std::uint32_t>(byte) << (8 * i);
    }
    return decoded;
  }
  float number(float value) {
    std::uint32_t bits;
    std::memcpy(&bits, &value, 4);
    bits = integer(bits, 4);
    std::memcpy(&value, &bits, 4);
    return value;
  }
};
Result transfer(FILE* file, std::uint8_t robot, bool writing) {
  Stream io{file, writing};
  const auto magic = io.integer(0x534e4f41U, 4);
  const auto version = io.integer(kVersion, 1);
  const auto identity = io.integer(robot, 1);
  const auto flags = io.integer(recording.flags, 1);
  const auto reserved = io.integer(0, 1);
  const auto points = io.integer(recording.pointCount, 2);
  const auto events = io.integer(recording.eventCount, 2);
  const auto duration = io.integer(recording.durationMs, 2);
  const auto reserved2 = io.integer(0, 2);
  if (!io.ok) return writing ? Result::Io : Result::Corrupt;
  if (magic != 0x534e4f41U || reserved || reserved2 || flags > kPrefix ||
      !points || points > kMaxPoints || events > kMaxEvents || duration > kMaxDurationMs)
    return Result::Corrupt;
  if (version != kVersion) return Result::Version;
  if (identity != robot) return Result::WrongRobot;
  if (!writing) {
    recording.robot = robot; recording.flags = static_cast<std::uint8_t>(flags);
    recording.pointCount = static_cast<std::uint16_t>(points);
    recording.eventCount = static_cast<std::uint16_t>(events);
    recording.durationMs = static_cast<std::uint16_t>(duration);
  }
  for (unsigned i = 0; i < points; ++i) {
    auto& p = recording.points[i];
    p.x = io.number(p.x); p.y = io.number(p.y); p.heading = io.number(p.heading);
    p.dwellMs = static_cast<std::uint16_t>(io.integer(p.dwellMs, 2));
    p.direction = static_cast<std::int8_t>(io.integer(static_cast<std::uint8_t>(p.direction), 1));
    p.flags = static_cast<std::uint8_t>(io.integer(p.flags, 1));
  }
  for (unsigned i = 0; i < events; ++i) {
    auto& e = recording.events[i];
    e.pointIndex = static_cast<std::uint16_t>(io.integer(e.pointIndex, 2));
    e.delayMs = static_cast<std::uint16_t>(io.integer(e.delayMs, 2));
    e.kind = static_cast<Kind>(io.integer(static_cast<unsigned>(e.kind), 1));
    e.value = static_cast<std::int8_t>(io.integer(static_cast<std::uint8_t>(e.value), 1));
  }
  const auto checksum = io.hash;
  const auto stored = io.integer(checksum, 4, false);
  if (!io.ok || stored != checksum || (!writing && std::fgetc(file) != EOF) || std::ferror(file))
    return writing ? Result::Io : Result::Corrupt;
  return validate(robot);
}
Result writeRecording(FILE* file, std::uint8_t robot) {
  const auto check = validate(robot);
  return check == Result::Ok ? transfer(file, robot, true) : check;
}
Result readRecording(FILE* file, std::uint8_t robot) {
  ready = false; capturing = false;
  const auto check = transfer(file, robot, false);
  ready = check == Result::Ok;
  if (!ready) recording.pointCount = recording.eventCount = 0;
  return check;
}

// Small synchronous scheduler, also used by host safety tests. Hardware callbacks
// are borrowed function pointers; no std::function allocations or second route.
struct Playback {
  std::uint32_t (*now)();
  Result (*health)();
  ShadowPoint (*pose)();
  void (*initialize)(const ShadowPoint&);
  bool (*drive)(const ShadowPoint&, const ShadowPoint&, std::int8_t, bool);
  void (*mechanism)(const ShadowEvent&);
  void (*stop)();
  void (*wait)();
};
Result runRoute(const Playback& io, std::uint8_t robot) {
  const auto finish = [&](Result why) { io.stop(); return why; };
  const auto check = validate(robot);
  if (check != Result::Ok) return finish(check);
  const auto initialHealth = io.health();
  if (initialHealth != Result::Ok) return finish(initialHealth);
  io.initialize(recording.points[0]);
  const auto began = io.now();
  std::uint16_t nextEvent = 0;
  ShadowPoint previous = io.pose();
  for (unsigned index = 0; index < recording.pointCount; ++index) {
    const auto& target = recording.points[index];
    const auto segmentStart = io.now();
    const auto initialDistance = distance(io.pose(), target);
    // Whole run and individual segment limits prevent a stuck controller from
    // continuing indefinitely; pose proximity, never the clock, advances motion.
    const auto timeout = static_cast<std::uint32_t>(3000 + initialDistance * 1000);
    bool reached = index == 0;
    std::uint32_t dwellStart = segmentStart;
    while (true) {
      const auto health = io.health();
      if (health != Result::Ok) return finish(health);
      const auto now = io.now();
      if (now - began > 180000 || now - segmentStart > timeout + target.dwellMs)
        return finish(Result::MotionFailed);
      const auto current = io.pose();
      if (!finite(current)) return finish(Result::InvalidPose);
      if (distance(previous, current) > 8 || std::fabs(angle(previous.heading, current.heading)) > 45)
        return finish(Result::PoseJump);
      previous = current;
      const auto& from = recording.points[index ? index - 1 : 0];
      const bool orient = index + 1 == recording.pointCount || target.dwellMs ||
                          from.direction != target.direction || distance(from, target) < 0.15F;
      const bool atPosition = distance(current, target) <= (orient ? 0.35F : 0.5F);
      const bool atHeading = std::fabs(angle(target.heading, current.heading)) <= 3;
      if (!reached && atPosition && (!orient || atHeading)) {
        reached = true; dwellStart = now;
      }
      if (reached) {
        // stop drive during dwell, but keep active mechanism output until its
        // next semantic transition. drive(..., true) is a zero-motion hold.
        if ((target.dwellMs || index + 1 == recording.pointCount) &&
            !io.drive(target, current, 0, true)) return finish(Result::Cancelled);
        while (nextEvent < recording.eventCount && recording.events[nextEvent].pointIndex == index &&
               recording.events[nextEvent].delayMs <= now - dwellStart) {
          const auto eventHealth = io.health();
          if (eventHealth != Result::Ok) return finish(eventHealth);
          io.mechanism(recording.events[nextEvent++]);
        }
        if (now - dwellStart >= target.dwellMs) break;
      } else if (!io.drive(target, current, from.direction, false)) {
        return finish(Result::MotionFailed);
      }
      io.wait();
    }
  }
  const auto health = io.health();
  return finish(health == Result::Ok && nextEvent != recording.eventCount ? Result::Corrupt : health);
}

} // namespace
} // namespace aon::shadow

#ifndef AON_SHADOW_HOST_TEST
#include "../../include/aon/tools/gui/gui.hpp"

namespace aon::shadow {
namespace {
constexpr std::uint8_t kRobot = USING_BIG_ROBOT ? 2 : 1;
struct Hardware {
  Drivetrain* drive = nullptr;
  Intake* intake = nullptr;
  pros::Controller* controller = nullptr;
  Piston* brooks = nullptr;
  Piston* auxiliary = nullptr;
  pros::Task* scan = nullptr;
  pros::Task* sort = nullptr;
  PurePursuit* follower = nullptr; // borrowed only during runSelected()
  MotionProfile* linear = nullptr;
  MotionProfile* lateral = nullptr;
  MotionProfile* angular = nullptr;
  bool paused = false, scanning = false;
};
static Hardware hardware;
// Recording/menu owns the buffer until autonomous requests playback. The
// existing safety task then owns both the buffer and Shadow motor commands;
// atomics transfer ownership and latch cancellation. No Shadow task or mutex.
enum class RunState : std::uint8_t { Idle, Requested, Playing, StorageRequested, Storing };
static std::atomic<RunState> runState{RunState::Idle};
static UiAction storageAction = UiAction::None;
static std::atomic<bool> interrupted{true}, armed{false}, shadowSelected{false};
struct MenuState {
  std::uint32_t displayedAt = 0;
  std::uint32_t armEpoch = 0;
  std::uint8_t slot = 1, loadedSlot = 0, confirmation = 0;
  bool stationary = true;
  std::int8_t direction = 0;
  int (*nativeAtArm)() = nullptr;
  const char* message = "SELECT SLOT / LOAD";
};
static MenuState menu;
// The GUI task never reads/mutates the shared recording. One action mailbox
// transfers requests to the existing competition loop; atomic status is display
// only. Playback still exclusively owns the route after Requested is published.
struct UiState {
  // Low byte = action; upper 24 bits = its generation. Bind confirmation to
  // that generation, so an older ARM cannot undo a newer GUI disarm.
  std::atomic<std::uint32_t> action{0}, epoch{0};
  std::atomic<bool> open{false}, recording{false};
  std::atomic<std::uint8_t> slot{1}, confirmation{0};
  std::atomic<std::uint16_t> duration{0}, points{0};
  std::atomic<float> x{0}, y{0}, heading{0};
  std::atomic<const char*> message{"SELECT SLOT / LOAD"};
};
static UiState ui;
static std::int32_t guiPress = -1; // GUI-task-only touch debounce
void publishUi() {
  ui.slot.store(menu.slot); ui.confirmation.store(menu.confirmation);
  ui.duration.store(recording.durationMs);
  ui.points.store(ready || capturing ? recording.pointCount : 0);
  if (ready || capturing) {
    ui.x.store(recording.points[0].x); ui.y.store(recording.points[0].y);
    ui.heading.store(recording.points[0].heading);
  }
  ui.recording.store(capturing);
  ui.message.store(capturing ? "RECORDING - DRIVE NORMALLY" : menu.message);
}

ShadowPoint robotPose() {
  const auto p = hardware.drive->getPose();
  return {static_cast<float>(p.x), static_cast<float>(p.y),
          static_cast<float>(std::remainder(p.theta, 360.0)), 0, 0, 0};
}
void pauseWriters() {
  if (!hardware.paused) hardware.scanning = hardware.intake->isScanning();
  hardware.scan->suspend();
  hardware.sort->suspend();
  hardware.paused = true;
}
void stopHardware() { // single route owner; no later asynchronous writer
  if (!hardware.drive) return;
  pauseWriters();
  hardware.drive->stop();
  hardware.intake->stopScan();
#if USING_BIG_ROBOT
  hardware.intake->stopReleasing();
#else
  hardware.intake->scorer(0); // Intake::stop() does not stop the lever motor
#endif
  hardware.intake->stop();
}
void safeStop() { stopHardware(); }
void resumeWriters(bool restoreScan = true) {
  if (!hardware.paused) return;
  if (restoreScan && hardware.scanning) hardware.intake->activateScan();
  hardware.scan->resume();
  hardware.sort->resume();
  hardware.paused = false;
}
void baseline() {
  stopHardware();
  hardware.brooks->deactivate();
  hardware.auxiliary->deactivate();
  hardware.intake->raiseCart();
#if !USING_BIG_ROBOT
  hardware.intake->lowerScorer();
  hardware.intake->closeTrapdoor();
  hardware.intake->resetLever();
#endif
}
Result health() {
  if (interrupted.load() || hardware.controller->get_digital(pros::E_CONTROLLER_DIGITAL_X))
    return Result::Cancelled;
  const auto state = runState.load();
  if ((state == RunState::Requested || state == RunState::Playing) &&
      !armCurrent(menu.armEpoch, ui.epoch.load(), 0)) return Result::Cancelled;
  if (pros::competition::is_disabled() || !pros::competition::is_autonomous()) return Result::Unsafe;
  if (!hardware.drive->hasFreshPose()) return Result::InvalidPose;
  return finite(robotPose()) ? Result::Ok : Result::InvalidPose;
}
void initializePose(const ShadowPoint& p) {
  if (health() != Result::Ok) return;
  baseline();
  hardware.drive->setPose(Pose(p.x, p.y, p.heading));
}
// Existing single-target PurePursuit / S-curve controls, fed directly from the
// array. No vector conversion, follower path copy, or blocking goToPose calls.
bool driveTarget(const ShadowPoint& target, const ShadowPoint& current, std::int8_t dir, bool hold) {
  if (health() != Result::Ok) return false;
  if (hold) { hardware.drive->stop(); return true; }
  const double error = distance(target, current);
  const double headingError = angle(target.heading, current.heading);
  if (error <= 0.35) {
    const double arc = DRIVE_WIDTH * M_PI * std::fabs(headingError) / 360;
    const double speed = std::min(hardware.angular->update(arc, 0.01), std::fabs(headingError) * 3);
    hardware.drive->holonomic(0, 0, std::copysign(speed, headingError));
    return true;
  }
#if USING_BIG_ROBOT
  // Field (X forward,Y right) -> robot-forward / robot-right. Unlike the
  // existing tank-only follow(), this preserves recorded H-drive strafing.
  const double dx = target.x - current.x, dy = target.y - current.y;
  const double radians = current.heading * M_PI / 180;
  const double forward = dx * std::cos(radians) + dy * std::sin(radians);
  const double sideways = -dx * std::sin(radians) + dy * std::cos(radians);
  const double linear = std::min(hardware.linear->update(std::fabs(forward), 0.01), std::fabs(forward) * 40);
  const double lateral = std::min(hardware.lateral->update(std::fabs(sideways), 0.01), std::fabs(sideways) * 40);
  const double turn = std::min(hardware.angular->update(DRIVE_WIDTH * M_PI * std::fabs(headingError) / 360, 0.01),
                               std::fabs(headingError) * 3);
  hardware.drive->holonomic(std::copysign(linear, forward), std::copysign(lateral, sideways),
                             std::copysign(turn, headingError));
  (void)dir; // signed robot-frame error naturally handles H-drive reverse/strafe
#else
  Pose virtualPose(current.x, current.y, current.heading + (dir < 0 ? 180 : 0));
  const auto output = hardware.follower->go(Pose(target.x, target.y, target.heading), virtualPose, 0.01);
  const double bearing = std::atan2(target.y - current.y, target.x - current.x) * 180 / M_PI;
  const double turnError = std::remainder(bearing - virtualPose.theta, 360.0);
  // PurePursuit::go currently drives forward even when facing away. Gate its
  // linear component by heading, and taper at the target rather than overshoot.
  const double linear = std::min((output.first + output.second) / 2, error * 40) *
                        std::max(0.0, std::cos(turnError * M_PI / 180));
  const double turn = std::copysign(std::min(std::fabs((output.first - output.second) / 2),
                                            std::fabs(turnError) * 3), turnError);
  const double signedLinear = dir < 0 ? -linear : linear;
  hardware.drive->arcade(signedLinear, turn);
#endif
  return true;
}
void applyMechanism(const ShadowEvent& e) {
  if (health() != Result::Ok) return;
  auto& intake = *hardware.intake;
  switch (e.kind) {
    case Kind::Intake:
      switch (static_cast<IntakeMode>(e.value)) {
        case IntakeMode::Idle: intake.stop(); break;
        case IntakeMode::Store: intake.store(); break;
#if !USING_BIG_ROBOT
        case IntakeMode::Corridor: intake.corridor(); break;
        case IntakeMode::Reject: intake.reject(); break;
#else
        case IntakeMode::Middle: intake.score(Intake::MIDDLE); break;
#endif
        case IntakeMode::Bottom: intake.score(Intake::BOTTOM); break;
        case IntakeMode::Top: intake.score(Intake::TOP); break;
        default: break; // validator rejects unsupported modes before playback
      }
      break;
    case Kind::Cart: e.value ? intake.dropCart() : intake.raiseCart(); break;
    case Kind::Brooks: e.value ? hardware.brooks->activate() : hardware.brooks->deactivate(); break;
#if USING_BIG_ROBOT
    case Kind::Sem: e.value ? hardware.auxiliary->activate() : hardware.auxiliary->deactivate(); break;
    case Kind::Sort:
      if (e.value) {
        // Sort exclusively owns intake output while enabled. The scan task
        // remains suspended; cancellation suspends sort before stopping motors.
        hardware.sort->resume();
        intake.setSortHeights(e.value == 1 ? Intake::TOP : Intake::MIDDLE);
        intake.startReleasing();
      } else {
        hardware.sort->suspend();
        intake.stopReleasing();
        intake.stop();
      }
      break;
#else
    case Kind::Arrow: e.value ? hardware.auxiliary->activate() : hardware.auxiliary->deactivate(); break;
    case Kind::ScorerHeight: e.value ? intake.raiseScorer() : intake.lowerScorer(); break;
    case Kind::Trapdoor: e.value ? intake.openTrapdoor() : intake.closeTrapdoor(); break;
    case Kind::Lever: e.value ? intake.extendLever() : intake.resetLever(); break;
#endif
    default: break;
  }
}
std::uint32_t robotTime() { return pros::millis(); }
void waitTick() { pros::delay(10); }
const char* label(Result value) {
  switch (value) {
    case Result::Ok: return "READY";
    case Result::Empty: return "EMPTY";
    case Result::Capacity: return "CAPACITY - B SAVE";
    case Result::Duration: return "60s LIMIT - B SAVE";
    case Result::InvalidPose: return "ODOMETRY ERROR";
    case Result::PoseJump: return "POSE JUMP";
    case Result::WrongRobot: return "WRONG ROBOT";
    case Result::Version: return "WRONG VERSION";
    case Result::Corrupt: return "INVALID FILE";
    case Result::NoSd: return "NO SD";
    case Result::Io: return "SD I/O FAILED";
    case Result::Unsafe: return "UNSAFE STATE";
    case Result::Locked: return "LOAD THEN ARM";
    case Result::Cancelled: return "CANCELLED";
    case Result::MotionFailed: return "MOTION FAILED";
  }
  return "ERROR";
}
void path(char* name, bool temporary) {
  std::snprintf(name, 32, "/usd/aon-shadow-%u.%s", menu.slot, temporary ? "tmp" : "bin");
}
Result loadSlot() {
  armed.store(false); menu.loadedSlot = 0; ready = false;
  if (!pros::usd::is_installed()) return Result::NoSd;
  char name[32]; path(name, false);
  FILE* file = std::fopen(name, "rb");
  if (!file) return Result::Io;
  auto check = readRecording(file, kRobot);
  if (std::fclose(file) != 0) { ready = false; check = Result::Io; }
  if (check == Result::Ok) menu.loadedSlot = menu.slot;
  return check;
}
Result saveSlot() {
  if (!ready) return result;
  if (!pros::usd::is_installed()) return Result::NoSd;
  char temporary[32], destination[32]; path(temporary, true); path(destination, false);
  FILE* file = std::fopen(temporary, "wb");
  if (!file) return Result::Io;
  auto check = writeRecording(file, kRobot);
  if (std::fflush(file) != 0) check = Result::Io;
  if (std::fclose(file) != 0) check = Result::Io;
  if (check != Result::Ok) return check; // destination untouched
  file = std::fopen(temporary, "rb");
  if (!file) return Result::Io;
  check = readRecording(file, kRobot); // reuse same buffer, never a full copy
  if (std::fclose(file) != 0) { ready = false; check = Result::Io; }
  if (check != Result::Ok) return check;
  if (std::rename(temporary, destination) != 0) return Result::Io;
  menu.loadedSlot = menu.slot;
  return Result::Ok;
}
void requestStorage(UiAction action) {
  // Transfer buffer ownership to the existing safety task. Unlike opcontrol /
  // disabled(), that task survives competition changes during SD operations.
  armed.store(false); safeStop(); storageAction = action;
  ui.message.store("SD BUSY - DRIVE LOCKED"); ui.recording.store(false);
  runState.store(RunState::StorageRequested);
}
} // namespace

void bind(Drivetrain& drive, Intake& intake, pros::Controller& controller,
          Piston& brooks, Piston& auxiliary, pros::Task& scan, pros::Task& sort) {
  hardware.drive = &drive; hardware.intake = &intake; hardware.controller = &controller;
  hardware.brooks = &brooks; hardware.auxiliary = &auxiliary;
  hardware.scan = &scan; hardware.sort = &sort;
}
void cancel() {
  interrupted.store(true); armed.store(false);
  // The safety task observes this latch within one 10 ms tick and executes the
  // final stop. It survives competition-task deletion, avoiding stranded locks.
  if (runState.load() == RunState::Idle) safeStop();
}
static void executeRequested();
void safetyPoll() { executeRequested(); }
bool selected() { return shadowSelected.load(); }
void prepareNativeControl() {
  cancel();
  while (runState.load() != RunState::Idle) pros::delay(10);
  capturing = false;
  // Configure() already selected the scan policy for the new competition mode.
  // Resume tasks without resurrecting a scan flag from the previous run.
  resumeWriters(false);
}
void enterDriverControl() {
  prepareNativeControl();
  ui.action.store(0); menu.confirmation = 0; publishUi();

}
void poll(float forward, float sideways, float turn) {
  if (!capturing) return;
  if (interrupted.load() || pros::competition::is_disabled() || pros::competition::is_autonomous() ||
      !hardware.drive->hasFreshPose()) { abortCapture(Result::InvalidPose); return; }
  menu.direction = std::fabs(forward) > 0.02F ? (forward > 0 ? 1 : -1) : 0;
  menu.stationary = std::fabs(forward) <= 0.02F && std::fabs(sideways) <= 0.02F && std::fabs(turn) <= 0.02F;
  const auto now = pros::millis();
  if (now - captureState.lastPoll < kPollMs) return;
  // Driver intent prevents slow rotation/strafe from being mistaken for noise.
  capture(robotPose(), now, menu.direction, false, menu.stationary);
}
void event(Kind kind, std::int8_t value) {
  if (!capturing) return;
  if (kind >= Kind::Count) { abortCapture(Result::Corrupt); return; }
  if (captureState.lastValue[static_cast<unsigned>(kind)] == value) return;
  if (interrupted.load() || !hardware.drive->hasFreshPose()) { abortCapture(Result::InvalidPose); return; }
  addEvent(kind, value, robotPose(), pros::millis(), menu.direction, menu.stationary);
}
void toggleEvent(Kind kind) {
  if (!capturing || kind >= Kind::Count) return;
  const auto last = captureState.lastValue[static_cast<unsigned>(kind)];
  event(kind, last == 1 ? 0 : 1);
}
bool controls() {
  if (!hardware.drive || runState.load() != RunState::Idle) return true;
  if (interrupted.load() && capturing) abortCapture(Result::Cancelled);
  const auto now = pros::millis();
  const auto request = ui.action.exchange(0);
  const auto action = static_cast<UiAction>(request & 255);
  const auto actionEpoch = request >> 8;
  const bool centered = std::abs(hardware.controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_X)) < 10 &&
                        std::abs(hardware.controller->get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y)) < 10 &&
                        std::abs(hardware.controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X)) < 10 &&
                        std::abs(hardware.controller->get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y)) < 10;
  if (action != UiAction::None) {
    if (capturing && action != UiAction::Save && action != UiAction::Cancel && action != UiAction::Open) {
      // Slot changes/load/delete/arm cannot steal the recording buffer.
      menu.message = "STOP/SAVE FIRST";
    } else if (action >= UiAction::Slot1 && action <= UiAction::Slot3) {
      armed.store(false); shadowSelected.store(false);
      menu.slot = static_cast<std::uint8_t>(action);
      menu.confirmation = 0;
      requestStorage(UiAction::Load);
    } else switch (action) {
      case UiAction::Open:
        if (!capturing) { safeStop(); menu.message = label(result); }
        break;
      case UiAction::Record:
        armed.store(false);
        if (pros::competition::is_disabled() || pros::competition::is_autonomous() || !centered)
          menu.message = "RECORD IN DRIVER / CENTER STICKS";
        else if (!hardware.drive->hasFreshPose()) menu.message = "ODOMETRY ERROR";
        else if (menu.confirmation != 1) {
          menu.confirmation = 1; menu.message = "RESET/OVERWRITE? TAP RECORD AGAIN";
        } else {
          menu.confirmation = 0; menu.loadedSlot = 0; menu.direction = 0;
          baseline(); interrupted.store(false);
          startCapture(robotPose(), now, kRobot);
          std::fill_n(captureState.lastValue, static_cast<unsigned>(Kind::Count), static_cast<std::int8_t>(0));
          if (capturing) resumeWriters(); else safeStop();
          menu.message = label(result);
        }
        break;
      case UiAction::Save:
        armed.store(false); menu.confirmation = 0;
        if (capturing) {
          if (hardware.drive->hasFreshPose()) finishCapture(robotPose(), now, menu.direction, menu.stationary);
          else abortCapture(Result::InvalidPose);
        }
        if (ready) requestStorage(UiAction::Save);
        else { safeStop(); menu.message = label(result); }
        break;
      case UiAction::Load:
        menu.confirmation = 0; requestStorage(UiAction::Load);
        break;
      case UiAction::Delete:
        armed.store(false); safeStop();
        if (menu.confirmation != 2) { menu.confirmation = 2; menu.message = "DELETE? TAP DELETE AGAIN"; }
        else {
          menu.confirmation = 0; requestStorage(UiAction::Delete);
        }
        break;
      case UiAction::Arm:
        if (!ready || menu.loadedSlot != menu.slot || validate(kRobot) != Result::Ok)
          menu.message = "LOAD/SAVE A VALID SLOT FIRST";
        else if (!centered || !hardware.drive->hasFreshPose() || pros::competition::is_autonomous())
          menu.message = "UNSAFE TO ARM";
        else if (menu.confirmation != 3) {
          menu.confirmation = 3; menu.message = "PLACE AT SHOWN POSE; TAP ARM AGAIN";
        } else {
          menu.confirmation = 0; menu.nativeAtArm = aon::gui->selectedAuton.routine;
          menu.armEpoch = actionEpoch;
          if (armCurrent(actionEpoch, ui.epoch.load(), ui.action.load())) {
            interrupted.store(false); armed.store(true); shadowSelected.store(true);
            menu.message = "ARMED - START AUTONOMOUS";
          } else menu.message = "NEW ACTION - ARM AGAIN";
        }
        break;
      case UiAction::Cancel:
        cancel();
        if (capturing) abortCapture(Result::Cancelled);
        menu.confirmation = 0; menu.message = "CANCELLED";
        break;
      case UiAction::Native:
        selectNative(); menu.confirmation = 0; menu.message = "NATIVE AUTON SELECTED";
        break;
      default: break;
    }
  }
  if (runState.load() != RunState::Idle) return true; // buffer belongs to SD worker
  if (!capturing && ui.recording.load() && action == UiAction::None) {
    safeStop();
    menu.message = label(result);
  }
  if (action != UiAction::None || now - menu.displayedAt >= 200) {
    publishUi(); menu.displayedAt = now;
  }
  // The GUI is passive while capturing; an arm locks ordinary driving too.
  if (armed.load() || (ui.open.load() && !capturing)) { hardware.drive->stop(); return true; }
  if (!capturing && hardware.paused && !pros::competition::is_disabled()) resumeWriters();
  return action != UiAction::None;
}

void selectNative() { ui.epoch.fetch_add(1); armed.store(false); shadowSelected.store(false); }
void openGui() {
  guiPress = pros::screen::touch_status().press_count; // consume navigation tap
  ui.open.store(true);
  std::uint32_t expected = 0;
  const auto request = ((ui.epoch.load() & kUiEpochMask) << 8) | static_cast<unsigned>(UiAction::Open);
  ui.action.compare_exchange_strong(expected, request);
}
static void drawButton(int x, int y, int width, const char* text, pros::Color color) {
  pros::screen::set_eraser(color);
  pros::screen::erase_rect(x, y, x + width - 1, y + 32);
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_SMALL, x + 8, y + 9, "%s", text);
}
bool guiTick() {
  // Called only by the existing GUI task; touch press-count prevents held taps
  // from confirming overwrite/delete/arming without a second physical tap.
  const auto touch = pros::screen::touch_status();
  if ((touch.touch_status == pros::E_TOUCH_PRESSED || touch.touch_status == pros::E_TOUCH_HELD) &&
      touch.press_count != guiPress) {
    guiPress = touch.press_count;
    const auto action = touchAction(touch.x, touch.y);
    if (action == UiAction::Back) {
      ui.open.store(false); return false; // navigation preserves explicit arm
    }
    const auto epoch = action == UiAction::None ? ui.epoch.load() : ui.epoch.fetch_add(1) + 1;
    if (action != UiAction::None) armed.store(false);
    if (action == UiAction::Cancel) { interrupted.store(true); armed.store(false); }
    if (runState.load() == RunState::Idle) {
      std::uint32_t expected = 0;
      const auto request = ((epoch & kUiEpochMask) << 8) | static_cast<unsigned>(action);
      if (action != UiAction::None) ui.action.compare_exchange_strong(expected, request);
    }
  }
  pros::screen::set_eraser(pros::Color::black); pros::screen::erase();
  pros::screen::set_pen(pros::Color::white);
  pros::screen::print(pros::E_TEXT_MEDIUM, 10, 10, "SHADOW AUTON");
  drawButton(180, 5, 120, "NATIVE AUTON", pros::Color::dark_gray);
  drawButton(385, 5, 90, "CANCEL", pros::Color::dark_red);
  for (unsigned slot = 1; slot <= 3; ++slot) {
    char name[16]; std::snprintf(name, sizeof(name), "SHADOW %u", slot);
    drawButton((slot - 1) * 160 + 5, 45, 150, name,
               ui.slot.load() == slot ? pros::Color::blue : pros::Color::dark_gray);
  }
  pros::screen::set_pen(pros::Color::white);
  const auto state = runState.load();
  pros::screen::print(pros::E_TEXT_SMALL, 10, 84, "%s",
      state == RunState::StorageRequested || state == RunState::Storing ? "SD BUSY - DRIVE LOCKED" :
      state != RunState::Idle ? "PLAYING" : armed.load() ? "ARMED - START AUTONOMOUS" : ui.message.load());
  if (ui.points.load()) {
    pros::screen::print(pros::E_TEXT_SMALL, 10, 102, "START X %.1f  Y %.1f  H %.1f deg",
                        ui.x.load(), ui.y.load(), ui.heading.load());
    pros::screen::print(pros::E_TEXT_SMALL, 10, 120, "%.1f seconds / %u points",
                        ui.duration.load() / 1000.0, ui.points.load());
  }
  const auto confirmation = ui.confirmation.load();
  drawButton(5, 140, 150, confirmation == 1 ? "CONFIRM RECORD" : "RECORD", pros::Color::dark_green);
  drawButton(165, 140, 150, "STOP / SAVE", pros::Color::dark_gray);
  drawButton(325, 140, 150, "LOAD", pros::Color::dark_gray);
  drawButton(5, 185, 150, confirmation == 2 ? "CONFIRM DELETE" : "DELETE", pros::Color::dark_red);
  drawButton(165, 185, 150, confirmation == 3 ? "CONFIRM POSE" : "ARM", pros::Color::dark_green);
  drawButton(325, 185, 150, "BACK", pros::Color::dark_gray);
  return ui.open.load();
}
int runSelected() {
  if (runState.load() != RunState::Idle || !armed.exchange(false) ||
      !armCurrent(menu.armEpoch, ui.epoch.load(), ui.action.load()) || !ready || menu.loadedSlot != menu.slot ||
      aon::gui->selectedAuton.routine != menu.nativeAtArm) {
    cancel(); return static_cast<int>(Result::Locked);
  }
  if (runState.load() != RunState::Idle || health() != Result::Ok) {
    cancel(); return static_cast<int>(Result::Unsafe);
  }
  runState.store(RunState::Requested);
  while (runState.load() != RunState::Idle) pros::delay(10);
  return static_cast<int>(result);
}
static void executeRequested() {
  auto storageExpected = RunState::StorageRequested;
  if (runState.compare_exchange_strong(storageExpected, RunState::Storing)) {
    if (storageAction == UiAction::Save) result = saveSlot();
    else if (storageAction == UiAction::Load) result = loadSlot();
    else if (storageAction == UiAction::Delete) {
      char name[32]; path(name, false);
      result = !pros::usd::is_installed() ? Result::NoSd : (std::remove(name) == 0 ? Result::Ok : Result::Io);
      if (result == Result::Ok) { ready = false; menu.loadedSlot = 0; }
    }
    menu.message = storageAction == UiAction::Delete && result == Result::Ok ? "DELETED" : label(result);
    safeStop(); publishUi();
    runState.store(RunState::Idle); // release ownership only after close/validation
    return;
  }
  auto expected = RunState::Requested;
  if (!runState.compare_exchange_strong(expected, RunState::Playing)) return;
  // Borrow stack-local controllers for this synchronous run, not another route.
  MotionProfile linear(120, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);
  MotionProfile lateral(100, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);
  MotionProfile angular(90, MAX_ACCEL, MAX_DECEL, MAX_ACCEL);
  PurePursuit follower(linear, angular, 1, 0.3, 3);
  hardware.linear = &linear; hardware.lateral = &lateral;
  hardware.angular = &angular; hardware.follower = &follower;
  const Playback callbacks{robotTime, health, robotPose, initializePose, driveTarget,
                           applyMechanism, safeStop, waitTick};
  result = runRoute(callbacks, kRobot);
  interrupted.store(true);
  hardware.linear = hardware.lateral = hardware.angular = nullptr;
  hardware.follower = nullptr;
  menu.message = label(result);
  publishUi();
  runState.store(RunState::Idle); // release buffer ownership AFTER final stop
}
} // namespace aon::shadow
#endif

#ifdef AON_SHADOW_HOST_TEST
// Host tests: compile THIS file with -DAON_SHADOW_HOST_TEST; no PROS headers.
#include <cassert>
#include <cstdio>
#include <limits>
#include <cstdlib>

// Avoid the Windows assertion dialog so failures are noninteractive.
#undef assert
#define assert(expr) do { if (!(expr)) { std::fprintf(stderr, "%s:%d: %s\n", __FILE__, __LINE__, #expr); std::exit(1); } } while (false)

using namespace aon::shadow;

static void fixture() {
  assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
  assert(addEvent(Kind::Intake, 1, {0, 0, 0, 0, 0, 0}, 0, 0) == Result::Ok);
  assert(capture({1, 0, 0, 0, 1, 0}, 50, 1) == Result::Ok);
  assert(finishCapture({2, 0, 0, 0, 1, 0}, 100, 1) == Result::Ok);
}
static void badFile(long offset, unsigned byte, Result expected, bool repairChecksum = false) {
  fixture();
  FILE* f = std::tmpfile();
  assert(f && writeRecording(f, 1) == Result::Ok);
  const auto size = std::ftell(f);
  assert(std::fseek(f, offset < 0 ? size + offset : offset, SEEK_SET) == 0);
  assert(std::fputc(static_cast<int>(byte), f) != EOF);
  if (repairChecksum) {
    std::rewind(f);
    std::uint32_t hash = 2166136261U;
    for (long i = 0; i < size - 4; ++i) hash = (hash ^ static_cast<unsigned>(std::fgetc(f))) * 16777619U;
    assert(std::fseek(f, size - 4, SEEK_SET) == 0);
    for (unsigned i = 0; i < 4; ++i) assert(std::fputc((hash >> (8 * i)) & 255, f) != EOF);
  }
  std::rewind(f);
  assert(readRecording(f, 1) == expected);
  assert(!ready && recording.pointCount == 0);
  assert(std::fclose(f) == 0);
}

struct FakePlayback {
  static std::uint32_t time, failureAt;
  static Result failure;
  static ShadowPoint position;
  static unsigned starts, stops, events, commands;
  static std::int8_t firstDirection;
  static bool failDrive, freeze, reverse;
  static std::uint32_t eventTimes[kMaxEvents];
  static std::int8_t eventValues[kMaxEvents];
  static std::uint32_t now() { return time; }
  static Result health() { return time >= failureAt ? failure : Result::Ok; }
  static ShadowPoint pose() { return position; }
  static void initialize(const ShadowPoint& p) { position = p; ++starts; }
  static bool drive(const ShadowPoint& p, const ShadowPoint&, std::int8_t dir, bool hold) {
    if (failDrive) return false;
    if (!hold) {
      if (!commands) firstDirection = dir;
      ++commands;
      reverse = reverse || dir < 0;
      if (!freeze) {
        const auto dist = distance(p, position);
        const auto fraction = dist > 0 ? std::min(1.0F, 0.25F / dist) : 1.0F;
        position.x += (p.x - position.x) * fraction;
        position.y += (p.y - position.y) * fraction;
        position.heading += std::clamp(angle(p.heading, position.heading), -3.0F, 3.0F);
      }
    }
    return true;
  }
  static void mechanism(const ShadowEvent& e) { eventTimes[events] = time; eventValues[events++] = e.value; }
  static void stop() { ++stops; }
  static void wait() { time += 10; }
  static Playback setup() {
    time = starts = stops = events = commands = 0;
    failureAt = std::numeric_limits<std::uint32_t>::max();
    failure = Result::Cancelled;
    position = {}; failDrive = freeze = reverse = false;
    firstDirection = 2;
    return {now, health, pose, initialize, drive, mechanism, stop, wait};
  }
};
std::uint32_t FakePlayback::time, FakePlayback::failureAt;
Result FakePlayback::failure;
ShadowPoint FakePlayback::position;
unsigned FakePlayback::starts, FakePlayback::stops, FakePlayback::events, FakePlayback::commands;
std::int8_t FakePlayback::firstDirection;
bool FakePlayback::failDrive, FakePlayback::freeze, FakePlayback::reverse;
std::uint32_t FakePlayback::eventTimes[kMaxEvents];
std::int8_t FakePlayback::eventValues[kMaxEvents];

int main() {
  // A newer tap must invalidate an older ARM even after its request is consumed.
  assert(armCurrent(7, 7, 0));
  assert(!armCurrent(7, 8, 0));
  assert(!armCurrent(7, 7, 1));
  assert(touchAction(10, 55) == UiAction::Slot1);
  assert(touchAction(170, 55) == UiAction::Slot2);
  assert(touchAction(330, 55) == UiAction::Slot3);
  assert(touchAction(30, 155) == UiAction::Record);
  assert(touchAction(180, 155) == UiAction::Save);
  assert(touchAction(350, 155) == UiAction::Load);
  assert(touchAction(30, 200) == UiAction::Delete);
  assert(touchAction(180, 200) == UiAction::Arm);
  assert(touchAction(350, 200) == UiAction::Back);
  assert(touchAction(440, 20) == UiAction::Cancel);
  assert(touchAction(200, 20) == UiAction::Native);
  assert(touchAction(-1, 155) == UiAction::None);
  assert(touchAction(160, 100) == UiAction::None);
  static_assert(sizeof(ShadowPoint) == 16);
  static_assert(sizeof(ShadowEvent) == 6);
  // The first leg after a start/dwell must inherit the driver's reverse intent.
  for (unsigned dwell : {0U, 500U}) {
    assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
    for (unsigned t = 50; t <= dwell; t += 50)
      assert(capture({0, 0, 0, 0, 0, 0}, t, 0) == Result::Ok);
    assert(capture({-1, 0, 0, 0, -1, 0}, dwell + 50, -1) == Result::Ok);
    assert(finishCapture({-2, 0, 0, 0, -1, 0}, dwell + 100, -1) == Result::Ok);
    const auto reverseCallbacks = FakePlayback::setup();
    assert(runRoute(reverseCallbacks, 1) == Result::Ok);
    assert(FakePlayback::firstDirection == -1);
  }
  assert(startCapture({0, 0, 359, 0, 1, 0}, 0, 1) == Result::Ok);
  assert(capture({0.1F, 0, 0, 0, 1, 0}, 10, 1) == Result::Ok);
  assert(recording.pointCount == 1); // sampling interval, heading wrap
  for (unsigned t = 50; t <= 5000; t += 50)
    assert(capture({t / 100.0F, 0, 0, 0, 1, 0}, t, 1) == Result::Ok);
  assert(recording.pointCount < 30); // online straight-line reduction
  assert(recording.points[0].x == 0);
  assert(recording.points[recording.pointCount - 1].x >= 49);
  assert(addEvent(Kind::Intake, 1, {50, 0, 0, 0, 1, 0}, 5000, 1) == Result::Ok);
  const auto anchor = recording.events[0].pointIndex;
  assert(addEvent(Kind::Intake, 1, {50, 0, 0, 0, 1, 0}, 5000, 1) == Result::Ok);
  assert(recording.eventCount == 1);
  assert(capture({50, 1, 90, 0, 1, 0}, 5050, 1) == Result::PoseJump);
  assert(!capturing);
  assert(recording.events[0].pointIndex == anchor);

  assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
  for (unsigned t = 50; t <= 1000; t += 50)
    assert(capture({0.01F, 0, 0, 0, 0, 0}, t, 0) == Result::Ok);
  assert(recording.pointCount == 1);
  assert(recording.points[0].dwellMs == 1000);
  assert(addEvent(Kind::Intake, 1, {0, 0, 0, 0, 0, 0}, 1000, 0) == Result::Ok);
  assert(recording.events[0].delayMs == 1000);
  for (unsigned t = 1050; t <= 1500; t += 50)
    assert(capture({0, 0, 0, 0, 0, 0}, t, 0) == Result::Ok);
  assert(addEvent(Kind::Intake, 0, {0, 0, 0, 0, 0, 0}, 1500, 0) == Result::Ok);
  assert(recording.events[1].delayMs == 1500);
  assert(finishCapture({0, 0, 0, 0, 0, 0}, 1500, 0) == Result::Ok);
  assert(validate(1) == Result::Ok);

  FILE* file = std::tmpfile();
  assert(file);
  assert(writeRecording(file, 1) == Result::Ok);
  assert(std::ftell(file) == 16 + 16 + 2 * 6 + 4);
  std::rewind(file);
  assert(readRecording(file, 1) == Result::Ok);
  assert(recording.eventCount == 2 && recording.points[0].dwellMs == 1500);
  std::rewind(file);
  assert(readRecording(file, 2) == Result::WrongRobot);
  std::fclose(file);

  assert(startCapture({0, 0, 0, 0, 1, 0}, 0, 1) == Result::Ok);
  assert(capture({1, 0, 0, 0, 1, 0}, 50, 1) == Result::Ok);
  assert(capture({1, 0, 0, 0, -1, 0}, 100, -1) == Result::Ok);
  assert(capture({0, 0, 0, 0, -1, 0}, 150, -1) == Result::Ok);
  assert(recording.points[recording.pointCount - 1].direction == -1);
  assert(finishCapture({0, 0, 0, 0, -1, 0}, 150, -1) == Result::Ok);
  assert(validate(1) == Result::Ok);

  // A reversal may already have moved by the next pose poll, with no dwell.
  assert(startCapture({0, 0, 0, 0, 1, 0}, 0, 1) == Result::Ok);
  assert(capture({1, 0, 0, 0, 1, 0}, 50, 1) == Result::Ok);
  assert(capture({0.5F, 0, 0, 0, -1, 0}, 100, -1) == Result::Ok);
  assert(finishCapture({0, 0, 0, 0, -1, 0}, 150, -1) == Result::Ok);
  assert(recording.points[1].x == 1 && recording.points[1].direction == -1);

  assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
  assert(finishCapture({0, 0, 0, 0, 0, 0}, 0, 0) == Result::Empty);
  assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
  assert(capture({std::numeric_limits<float>::quiet_NaN(), 0, 0, 0, 0, 0}, 50, 0)
         == Result::InvalidPose);
  assert(!capturing);

  // Maximum capacities retain the prefix without wrapping or losing anchors.
  assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
  for (unsigned i = 1; i < kMaxPoints; ++i)
    assert(capture({static_cast<float>(i), 0, static_cast<float>((i % 2) * 4), 0, 1, 0}, i * 50, 1) == Result::Ok);
  assert(recording.pointCount == kMaxPoints);
  assert(capture({600, 0, 0, 0, 1, 0}, 30000, 1) == Result::Capacity);
  assert(!capturing && ready && recording.points[0].x == 0 && recording.points[599].x == 599);
  assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
  for (unsigned i = 0; i < kMaxEvents; ++i)
    assert(addEvent(Kind::Lever, static_cast<std::int8_t>(i % 2), {0, 0, 0, 0, 0, 0}, 0, 0) == Result::Ok);
  assert(addEvent(Kind::Lever, 0, {0, 0, 0, 0, 0, 0}, 0, 0) == Result::Capacity);
  assert(!capturing && ready && recording.eventCount == 64);
  assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
  for (unsigned t = 50; t < 60000; t += 50) assert(capture({0, 0, 0, 0, 0, 0}, t, 0) == Result::Ok);
  assert(capture({0, 0, 0, 0, 0, 0}, 60000, 0) == Result::Duration);
  assert(!capturing && recording.pointCount == 1 && recording.points[0].dwellMs == 60000);
  assert(startCapture({0, 0, 0, 0, 0, 0}, 0, 1) == Result::Ok);
  assert(capture({0, 0, 0, 0, 0, 0}, 201, 0) == Result::InvalidPose);

  badFile(0, 0, Result::Corrupt); // magic
  badFile(4, 99, Result::Version);
  badFile(5, 2, Result::WrongRobot);
  badFile(9, 255, Result::Corrupt); // point count
  badFile(10, 65, Result::Corrupt); // event count
  badFile(7, 1, Result::Corrupt); // reserved byte
  badFile(-1, 99, Result::Corrupt); // checksum
  badFile(35, 0x7f, Result::Corrupt, true); // infinity X with valid checksum
  badFile(31, 99, Result::Corrupt, true); // invalid point flags
  fixture();
  FILE* source = std::tmpfile();
  assert(source && writeRecording(source, 1) == Result::Ok);
  const auto fileSize = std::ftell(source);
  for (long length = 0; length < fileSize; ++length) {
    FILE* truncated = std::tmpfile();
    assert(truncated);
    std::rewind(source);
    for (long i = 0; i < length; ++i) assert(std::fputc(std::fgetc(source), truncated) != EOF);
    std::rewind(truncated);
    assert(readRecording(truncated, 1) != Result::Ok && !ready);
    assert(std::fclose(truncated) == 0);
  }
  assert(std::fclose(source) == 0);
  fixture();
  source = std::tmpfile();
  assert(source && writeRecording(source, 1) == Result::Ok);
  assert(std::fputc(1, source) != EOF); // trailing garbage
  std::rewind(source);
  assert(readRecording(source, 1) == Result::Corrupt);
  assert(std::fclose(source) == 0);

  // Exact worst-case serialization budget, and failed writes are reported.
  recording.pointCount = 600; recording.eventCount = 64;
  recording.durationMs = 60000; recording.robot = 1; recording.flags = 0;
  for (unsigned i = 0; i < kMaxPoints; ++i)
    recording.points[i] = {static_cast<float>(i), 0, 0, 0, 1, 0};
  for (unsigned i = 0; i < kMaxEvents; ++i)
    recording.events[i] = {0, 0, Kind::Intake, static_cast<std::int8_t>(i % 2)};
  source = std::tmpfile();
  assert(source && writeRecording(source, 1) == Result::Ok);
  assert(std::ftell(source) == 10004);
  assert(std::fclose(source) == 0);
  source = std::fopen(__FILE__, "rb");
  assert(source && writeRecording(source, 1) == Result::Io);
  assert(std::fclose(source) == 0);

  // Progress-triggered moving event followed by ordered stationary events.
  fixture();
  recording.points[recording.pointCount - 1].dwellMs = 300;
  recording.durationMs = 400;
  recording.points[1].direction = -1;
  recording.events[1] = {static_cast<std::uint16_t>(recording.pointCount - 1), 100, Kind::Intake, 0};
  recording.events[2] = {static_cast<std::uint16_t>(recording.pointCount - 1), 200, Kind::Intake, 1};
  recording.eventCount = 3;
  auto callbacks = FakePlayback::setup();
  assert(runRoute(callbacks, 1) == Result::Ok);
  assert(FakePlayback::starts == 1 && FakePlayback::stops == 1 && FakePlayback::reverse);
  assert(FakePlayback::events == 3 && FakePlayback::eventValues[0] == 1 &&
         FakePlayback::eventValues[1] == 0 && FakePlayback::eventValues[2] == 1);
  assert(FakePlayback::eventTimes[2] - FakePlayback::eventTimes[1] == 100);
  for (auto failure : {Result::Cancelled, Result::Unsafe, Result::InvalidPose}) {
    callbacks = FakePlayback::setup(); FakePlayback::failureAt = 20; FakePlayback::failure = failure;
    assert(runRoute(callbacks, 1) == failure);
    assert(FakePlayback::stops == 1 && FakePlayback::events == 1);
  }
  callbacks = FakePlayback::setup(); FakePlayback::failDrive = true;
  assert(runRoute(callbacks, 1) == Result::MotionFailed && FakePlayback::stops == 1);
  callbacks = FakePlayback::setup(); FakePlayback::freeze = true;
  assert(runRoute(callbacks, 1) == Result::MotionFailed && FakePlayback::stops == 1);
  callbacks = FakePlayback::setup();
  assert(runRoute(callbacks, 2) == Result::WrongRobot && FakePlayback::stops == 1 && FakePlayback::starts == 0);
  recording.points[0].x = std::numeric_limits<float>::infinity();
  callbacks = FakePlayback::setup();
  assert(runRoute(callbacks, 1) == Result::Corrupt && FakePlayback::stops == 1 && FakePlayback::commands == 0);
  std::printf("Shadow tests passed; point=%zu event=%zu recording=%zu capture state=%zu\n",
              sizeof(ShadowPoint), sizeof(ShadowEvent), sizeof(ShadowRecording), sizeof(CaptureState));
}

#endif
