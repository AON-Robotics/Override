#pragma once

#ifndef AON_PI_PROTOCOL_HPP_
#define AON_PI_PROTOCOL_HPP_

/**
 * \file protocol.hpp
 *
 * \brief Line protocol between the Raspberry Pi and the V5 brain.
 *
 * This file and protocol.cpp are portable on purpose: they must not include
 * any PROS header. The same code is compiled on a laptop for the brain
 * simulator (RaspberryPi/sim) so that the parser that gets tested is the parser
 * that ships. All platform access goes through `Link::Hooks` and `Robot`.
 *
 * Wire format (see RaspberryPi/docs/serial-protocol.md for the full spec):
 *
 *   Pi -> brain   C,<seq>,<VERB>[,<arg>...]*HH
 *                 R,<inches> / N,0           (legacy red_tracker packets)
 *   brain -> Pi   @A,<seq>*HH                (ack: accepted)
 *                 @P,<seq>,k=v;k=v...*HH    (partial: more fields of a
 *                                            long @D, sent before it)
 *                 @D,<seq>,<status>[,k=v;k=v...]*HH   (done)
 *                 @H,k=v;k=v...*HH           (heartbeat, 5 Hz while linked)
 *                 @E,<code>[,k=v...]*HH      (event: abort, link lost, ...)
 *
 * `HH` is the XOR of every byte before the `*`, as two uppercase hex digits.
 *
 * \warning The Pi is an optional add-on. Nothing in here may block, throw or
 * crash the brain program, whatever bytes arrive.
 */

#include <atomic>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace aon::pi {

/// Protocol version reported by PING. Bump when the wire format changes.
constexpr int PROTOCOL_VERSION = 1;

/// Longest Pi -> brain line accepted, without the newline. Anything longer is
/// dropped and answered with `err,code=too_long`.
constexpr std::size_t MAX_RX_LINE = 128;

/// Longest brain -> Pi line body. Longer replies are split into `@P` partial
/// lines followed by the final `@D`, because a long line can overflow the
/// brain's USB output buffer and arrive truncated.
constexpr std::size_t MAX_TX_BODY = 200;

// ----------------------------------------------------------------------------
// Framing helpers
// ----------------------------------------------------------------------------

/// XOR checksum of `body` (the bytes before `*`).
std::uint8_t checksum(const std::string &body);

/// Appends `*HH` to `body`.
std::string withChecksum(const std::string &body);

/// Splits `line` into body and checksum. Returns false when the line has no
/// `*HH` suffix or it does not match. `hadChecksum` tells the two apart.
bool verifyChecksum(const std::string &line, std::string &body, bool &hadChecksum);

/// Makes a value safe to put in a `k=v` field: protocol separators
/// (`, ; = * \r \n`) become `_`.
std::string sanitize(const std::string &value);

// ----------------------------------------------------------------------------
// KV: builder for the `k=v;k=v` result fields
// ----------------------------------------------------------------------------

/// Builds the `k=v;k=v` payload of a reply. Keys may contain dots to nest
/// (`odom.x`, `motors.L.p11.temp`); the Pi turns them into nested objects.
class KV {
 public:
  KV &add(const std::string &key, double value, int decimals = 2);
  KV &add(const std::string &key, int value);
  KV &add(const std::string &key, long value);
  KV &add(const std::string &key, unsigned long value);
  KV &add(const std::string &key, bool value);
  KV &add(const std::string &key, const std::string &value);
  KV &add(const std::string &key, const char *value);

  /// Copies every field of `other`, prefixing its keys with `prefix.`.
  KV &merge(const KV &other, const std::string &prefix = "");

  bool empty() const { return fields.empty(); }
  const std::string &str() const { return fields; }

 private:
  std::string fields;
  void append(const std::string &key, const std::string &value);
};

// ----------------------------------------------------------------------------
// Commands
// ----------------------------------------------------------------------------

/// The arguments of one command, after the verb.
class Args {
 public:
  explicit Args(std::vector<std::string> values) : values(std::move(values)) {}

  std::size_t size() const { return values.size(); }
  const std::string &at(std::size_t i) const { return values[i]; }

  /// Parses argument `i` as a finite number. False when missing or not one.
  bool number(std::size_t i, double &out) const;

 private:
  std::vector<std::string> values;
};

enum class Status { OK, ERR, ABORTED, TIMEOUT };

const char *statusName(Status status);

/// What a command handler returns.
struct Result {
  Status status = Status::OK;
  KV fields;

  static Result ok(KV fields = KV()) { return {Status::OK, std::move(fields)}; }
  static Result timeout(KV fields = KV()) { return {Status::TIMEOUT, std::move(fields)}; }

  /// An error with a machine-readable `code` and a human `msg`. The message is
  /// what the user ends up reading, so make it say what to do.
  static Result error(const std::string &code, const std::string &msg) {
    Result r{Status::ERR, KV()};
    r.fields.add("code", code).add("msg", msg);
    return r;
  }
};

/// IMMEDIATE commands run in the reader task and must return within a few
/// milliseconds (reads, STOP). MOTION commands are queued to the worker task,
/// one at a time, and give the Pi control of the drivetrain while they run.
enum class Kind { IMMEDIATE, MOTION };

using Handler = std::function<Result(const Args &)>;
using SensorFn = std::function<void(KV &)>;

// ----------------------------------------------------------------------------
// Link: the brain side of the protocol
// ----------------------------------------------------------------------------

/**
 * \brief Parses Pi lines, dispatches commands and enforces link safety.
 *
 * Three tasks call into it on the brain:
 *   - reader task:    `feed()` for every byte from stdin
 *   - worker task:    `runPendingMotion()` in a loop
 *   - ticker task:    `tick()` every ~20 ms (heartbeat + deadman)
 * plus `abort()` from anywhere (driver joystick, X button).
 */
class Link {
 public:
  struct Hooks {
    std::function<std::uint32_t()> millis;
    /// Writes one complete line (already terminated). Must never block.
    std::function<void(const std::string &)> writeLine;
    /// Mutual exclusion for the Link state (pros::Mutex on the brain).
    std::function<void()> lock;
    std::function<void()> unlock;
    /// Human-readable log line for the brain console / LCD.
    std::function<void(const std::string &)> log;
    /// Called with the Link lock held when a motion must stop now. Must make
    /// the running motion return quickly and stop the motors.
    std::function<void()> abortMotion;
    /// Called with the Link lock held right before and right after a
    /// motion command, to re-arm the drivetrain's abort flag.
    std::function<void()> clearAbort;
    /// Extra heartbeat fields (pose, battery, mode).
    std::function<void(KV &)> heartbeatFields;
  };

  struct Config {
    /// Abort a Pi motion when nothing arrives from the Pi for this long.
    std::uint32_t deadmanMs = 1000;
    /// Heartbeat period while the Pi is talking to us.
    std::uint32_t heartbeatMs = 200;
    /// Stop sending heartbeats this long after the last line from the Pi, so
    /// a laptop running `pros terminal` is not flooded when no Pi is attached.
    std::uint32_t linkIdleMs = 5000;
  };

  Link(Hooks hooks, Config config);
  explicit Link(Hooks hooks) : Link(std::move(hooks), Config()) {}

  // --- Registration (call before the tasks start) ---------------------------

  /// Registers a verb. `minArgs`/`maxArgs` are checked before `fn` runs.
  void registerCommand(const std::string &verb, Kind kind, int minArgs, int maxArgs, Handler fn);

  /// Registers a sensor for the SENSORS command. `fn` fills fields without
  /// the prefix; they are sent as `<name>.<key>`.
  void registerSensor(const std::string &name, SensorFn fn);

  /// Called for legacy `R,<inches>` / `N,0` packets from red_tracker.
  void onLegacyPacket(std::function<void(char tag, int value)> fn);

  // --- Task entry points ----------------------------------------------------

  /// Reader task: one byte from the Pi.
  void feed(char c);
  /// Reader task: a complete line without the newline (used by `feed`).
  void handleLine(const std::string &line);

  /// Worker task: runs the queued motion, if any. Returns true if it did.
  bool runPendingMotion();

  /// Ticker task: heartbeat and deadman. Call every 10-50 ms.
  void tick();

  // --- Safety ---------------------------------------------------------------

  /// Aborts the pending/running Pi motion, if there is one. Safe to call from
  /// any task and at any time; does nothing when the Pi has no control.
  void abort(const std::string &reason);

  /// True while a Pi motion is queued or running. Driver code must not
  /// command the drivetrain while this is true.
  bool hasControl() const { return control.load(); }

  /// Sends an `@E` event line and logs it.
  void event(const std::string &code, const KV &fields = KV());

  // --- Introspection (SENSORS, heartbeat) -----------------------------------

  /// Fills every registered sensor, or only `name` when not empty. Returns
  /// false when `name` is unknown.
  bool readSensors(KV &out, const std::string &name = "");
  std::vector<std::string> sensorNames() const;

  struct Stats {
    std::uint32_t rxLines = 0;
    std::uint32_t badChecksum = 0;
    std::uint32_t tooLong = 0;
    std::uint32_t unknownVerb = 0;
    std::uint32_t aborts = 0;
    std::uint32_t deadmanTrips = 0;
  };
  Stats stats() const;

  /// Milliseconds since the last line from the Pi, or -1 if never.
  long msSinceLastRx() const;

  /// Verb of the motion that holds control, or "-" when idle.
  std::string busyVerb() const;

  /// The platform clock, in milliseconds.
  std::uint32_t now() const { return hooks.millis(); }

 private:
  struct Command {
    std::string verb;
    Kind kind;
    int minArgs;
    int maxArgs;
    Handler fn;
  };
  struct Pending {
    bool valid = false;
    long seq = 0;
    std::string verb;
    std::vector<std::string> args;
  };
  struct Sensor {
    std::string name;
    SensorFn fn;
  };

  void reply(long seq, const Result &result);
  void ack(long seq);
  void send(const std::string &body);
  const Command *find(const std::string &verb) const;

  class Guard {
   public:
    explicit Guard(const Hooks &h) : h(h) { if (h.lock) h.lock(); }
    ~Guard() { if (h.unlock) h.unlock(); }
   private:
    const Hooks &h;
  };

  Hooks hooks;
  Config config;
  std::vector<Command> commands;
  std::vector<Sensor> sensors;
  std::function<void(char, int)> legacyFn;

  // Reader-task only.
  std::string rxBuffer;
  bool rxOverflow = false;

  // Guarded by the lock.
  Pending pending;
  bool running = false;
  long runningSeq = 0;
  std::string runningVerb;
  std::string abortReason;
  bool everRx = false;
  std::uint32_t lastRxMs = 0;
  std::uint32_t lastHeartbeatMs = 0;
  bool linkActive = false;
  Stats counters;

  std::atomic<bool> control{false};
};

}  // namespace aon::pi

#endif  // AON_PI_PROTOCOL_HPP_
