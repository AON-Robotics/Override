# Raspberry Pi link

The brain side of the LLM debugging bridge. A Raspberry Pi plugged into the
brain's USB port sends commands (move, turn, read sensors, ...). This code runs
them with Override's own drivetrain and odometry and sends back what happened.
The Pi side lives in the RaspberryPi repo (`bridge/`, protocol in
`docs/serial-protocol.md`).

| File | What |
| --- | --- |
| `protocol.hpp` / `src/aon/pi/protocol.cpp` | Line framing, checksums, the `Link` (dispatch, busy, abort, deadman, heartbeat). No PROS. |
| `commands.hpp` / `src/aon/pi/commands.cpp` | The command set (PING, STATUS, SENSORS, STOP, MOVE, TURN, PROBE, RESET_ODOM) against an abstract `Robot`. No PROS. |
| `pi-link.hpp` / `src/aon/pi/pi-link.cpp` | `Robot` built on Override's `Drivetrain` and `Odometry`, the sensors, the three PROS tasks. |

`protocol.cpp` and `commands.cpp` must stay free of PROS headers: the
RaspberryPi repo compiles them on a laptop for its brain simulator and
end-to-end tests.

## Wiring (already done)

- `main.cpp`
  - `initialize()` calls `aon::pi::start(drivetrain, odometry, mainController)`.
  - `opcontrol()` runs the driver code only when `!aon::pi::hasControl()`.
    Otherwise it calls `aon::pi::checkDriverOverride()`.
- `globals.hpp`: `aon::STOP()`, the X button, also aborts Pi motion.
- `drivetrain.hpp`: every motion loop also exits on `requestAbort()`. The flag
  is cleared again once the Pi's motion ends, so autons are never affected.

## The Pi is an add-on

- With no Pi attached nothing changes: no control, no heartbeat output.
- The Pi can only move the robot in driver control. Motion is refused while
  disabled, in autonomous, or while the IMU is calibrating.
- A joystick move, the X button, a `STOP`, or 1 s without data from the Pi
  aborts the motion and hands control back to the driver.
- **COBS:** `start()` switches stdout from COBS-framed streams to plain lines
  so the Pi can read them. If `pros terminal` shows odd characters, use
  `pros terminal --raw`.

## Adding a sensor

In `registerSensors()` in `src/aon/pi/pi-link.cpp`:

```cpp
static pros::Distance frontDistance(5);
link.registerSensor("front_distance", [](KV &kv) {
  kv.add("mm", static_cast<int>(frontDistance.get_distance()));
});
```

It shows up in the Pi's `read_sensors` tool with no change on the Pi.

## Sensor packets from the Pi (OTOS, red target)

The Pi's other programs write to the same USB port as the bridge server:
- `vexpi` streams `O,<x>,<y>,<heading>` at 50 Hz.
- `red_tracker` sends `R,<in>` / `N,0`.

The reader task here is the **only** `stdin` reader on the brain, so those
packets reach Override through this module:

| Use | How |
| --- | --- |
| OTOS pose in odometry fusion | `aon::pi::latestOtosPose(pose, ageMs)`. Treat it as stale after `aon::pi::OTOS_TIMEOUT_MS` (300 ms). |
| Any packet, by tag | `link.registerPacket('X')` in `commands.cpp`, then `link.latestPacket('X', fields, ageMs)` |
| From the Pi / the LLM | the `pi_otos` and `pi_target` sensors (`read_sensors`) |

Sensor packets are never answered, and they never keep the deadman alive.
Only bridge commands do, so a dead bridge server still aborts the Pi's
motion while `vexpi` keeps streaming.

> **Merging `SparkSensor-test`:** that branch reads OTOS with its own
> `fgetc(stdin)` loop in `Odometry::initialize()`. With this module running,
> two readers would split the bytes between them and both would break.
> Replace that loop with `aon::pi::latestOtosPose()`; it gives the same pose
> and age the loop produced.
