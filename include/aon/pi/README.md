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
