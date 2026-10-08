#pragma once

#include <cstddef>
#include <cstdint>

/**
 * Lightweight Shadow driver guide (kept here to respect the five-file budget).
 *
 * Insert an SD card. On the Brain tap AUTONS -> SHADOW. Tap SHADOW 1/2/3
 * to select/load a slot; LOAD reloads it. In driver control with centered
 * sticks, tap RECORD then CONFIRM RECORD (confirms mechanism reset/overwrite).
 * Drive normally while the Brain screen remains available. Tap STOP / SAVE
 * to stop and write once. DELETE requires a second CONFIRM DELETE tap.
 * Automatic limits retain a bounded prefix in RAM; tap STOP / SAVE to save it.
 * Errors appear on the Brain; failed captures cannot be saved. No controller
 * combination opens or starts Shadow. RECORD resets supported mechanisms to
 * a known retracted/idle baseline for absolute transition events after reboot.
 *
 * The screen shows duration, point count, starting X/Y (inches), and heading
 * (clockwise degrees; X forward, Y right). Physically place the robot there,
 * tap ARM, then CONFIRM POSE. Physically home the lever first: Configure()
 * tares its encoder; baseline commands are not homing/calibration.
 * BACK preserves the arm; start autonomous to consume it once and set odometry
 * to point 0. Shadow NEVER navigates to the start. Disabled/driver entry, slot
 * changes, deletion/loading, or selecting a native auton invalidate old arms.
 * After disabling, reopen SHADOW and explicitly rearm for the next run.
 * NATIVE AUTON returns dispatch to the Brain's native selection. CANCEL on
 * the Brain or Controller X during autonomous latches cancellation. X retains
 * its ordinary driver control outside playback. Failure/disable/cancel stops
 * drivetrain AND active intake/lever, suspends asynchronous intake writers,
 * and never resumes the route. The existing safety task owns replay so deleted
 * competition tasks cannot bypass its final stop. No extra Shadow task.
 * GUI touches use an atomic request mailbox; the control loop owns capture,
 * the existing safety task owns SD/replay, and the GUI draws atomic metadata.
 * SD operations survive competition-mode changes and keep driving locked.
 * Host tests remain in shadow.cpp behind AON_SHADOW_HOST_TEST to keep five files.
 * Recorded data: reduced odometry poses, drive direction, dwell durations,
 * and semantic mechanism transitions with waypoint/dwell-relative timing.
 * Raw joystick axes, motor commands, and extra sensor telemetry are deliberately
 * omitted by the lightweight design; this does not satisfy telemetry requirements
 * of the broader issue. Fixed 50 ms pose polling does not imply fixed-rate storage.
 * Replay uses PurePursuit::go() and motion profiles on the small robot; H-drive
 * uses profiled robot-frame position/heading errors. Both advance by odometry
 * feedback rather than recorded motor timing. Events follow waypoint progress.
 * Drive smooth paths, make clear direction changes, pause for stationary actions,
 * and avoid wheel spin. Recorded speed is not reproduced; tune and measure final
 * position/heading, path consistency and event timing over repeated physical
 * runs at different battery levels before treating the broader issue as complete.
 * MSVC: cl /std:c++17 /EHsc /W4 /WX /DAON_SHADOW_HOST_TEST
 *          /D_CRT_SECURE_NO_WARNINGS src/aon/shadow.cpp
 * GCC: g++ -std=c++17 -Wall -Wextra -Werror -DAON_SHADOW_HOST_TEST
 *          src/aon/shadow.cpp -o /tmp/shadow-test
 *
 * SD: /usd/aon-shadow-N.bin, explicitly serialized little-endian IEEE binary32.
 * 16-byte header: "AONS", version u8, robot u8, flags u8, reserved u8,
 * pointCount u16, eventCount u16, durationMs u16, reserved u16.
 * Then 16 bytes/point, 6 bytes/event, and 4-byte FNV-1a checksum of all preceding
 * bytes. Max file = 16 + 600*16 + 64*6 + 4 = 10,004 bytes. A temporary file is
 * flushed/closed/read back before rename; no A/B recovery or continuous writes.
 * Verify SD rename replacement and power-loss behavior on the actual V5/FAT.
 *
 * Physical gates: record; save; reboot; load/inspect metadata; 2-3 ft straight;
 * 90-degree turn; L path; reverse; dwell; X cancel; disable; one event; several
 * events; full 15-second auton; longer only after repeatability is proven.
 * Odometry, traction, start placement, field and mechanisms determine accuracy.
 * The 600-point/64-event limits stop capture, never wrap. Highly curved or busy
 * 60-second recordings may reach capacity. Position/heading thresholds and the
 * conservative replay speed/tolerances require tuning on BOTH robots.
 */
namespace aon::shadow {

constexpr std::uint16_t kMaxPoints = 600;
constexpr std::uint16_t kMaxEvents = 64;
constexpr std::uint32_t kPollMs = 50;
constexpr std::uint32_t kKeyframeMs = 300;
constexpr std::uint32_t kMaxDurationMs = 60000;
constexpr float kPositionInches = 0.75F;
constexpr float kHeadingDegrees = 3.0F;

// direction describes the OUTGOING segment; 0 also permits turning/strafe.
// dwellMs is spent here BEFORE the outgoing segment. flags pins an online anchor.
struct ShadowPoint {
  float x, y, heading;
  std::uint16_t dwellMs;
  std::int8_t direction;
  std::uint8_t flags;
};
enum class Kind : std::uint8_t {
  Intake, ScorerHeight, Cart, Trapdoor, Lever, Brooks, Sem, Arrow, Sort, Count
};
enum class IntakeMode : std::int8_t { Idle, Store, Corridor, Reject, Bottom, Top, Middle };
struct ShadowEvent {
  std::uint16_t pointIndex;
  std::uint16_t delayMs; // offset WITHIN this point's dwell, zero during motion
  Kind kind;
  std::int8_t value;
};
struct ShadowRecording {
  ShadowPoint points[kMaxPoints];
  ShadowEvent events[kMaxEvents];
  std::uint16_t pointCount, eventCount, durationMs;
  std::uint8_t robot, flags;
};
static_assert(sizeof(ShadowPoint) == 16, "Shadow trajectory RAM budget");
static_assert(sizeof(ShadowEvent) == 6, "Shadow event RAM budget");
static_assert(sizeof(ShadowRecording) == 9992, "One shared recording buffer");

enum class Result : std::uint8_t {
  Ok, Empty, Capacity, Duration, InvalidPose, PoseJump, Corrupt, Version,
  WrongRobot, Io, NoSd, Unsafe, Locked, Cancelled, MotionFailed
};

} // namespace aon::shadow

#ifndef AON_SHADOW_HOST_TEST
#include "drivetrain/drivetrain.hpp"
#include "intake/intake.hpp"
#include "piston/piston.hpp"

namespace aon::shadow {
// The existing main.cpp owns hardware/tasks; Shadow never creates an RTOS task.
void bind(Drivetrain& drive, Intake& intake, pros::Controller& controller,
          Piston& brooks, Piston& auxiliary, pros::Task& scan, pros::Task& sort);
bool controls(); // true means menu has the drive; call before normal driver code
void openGui(); // GUI task: hardware actions go through the request mailbox
bool guiTick(); // GUI task: draw/touch Shadow screen; false returns to AUTONS
void selectNative(); // invalidate Shadow selection/arm when selecting native auton
void poll(float forward, float sideways = 0, float turn = 0);
void event(Kind kind, std::int8_t value);
void toggleEvent(Kind kind);
void enterDriverControl();
void prepareNativeControl(); // release background writers before native auton
void cancel(); // safe from disabled()/the existing safety task; latches cancellation
void safetyPoll();
bool selected();
int runSelected();
} // namespace aon::shadow
#endif
