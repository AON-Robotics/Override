# Raspberry Pi link

The brain side of the LLM debugging bridge. A Raspberry Pi plugged into the
brain's USB port sends commands (move, turn, read sensors, ...). This code runs
them with Override's own drivetrain and odometry and sends back what happened.
The Pi side lives in the RaspberryPi repo (`bridge/`, protocol in
`docs/serial-protocol.md`).

## What changed: before and after

**Before**, the link was one-way. The Pi only sent sensor numbers (`O,...`
from `vexpi`'s OTOS, `R,...` from `red_tracker`); the brain read them and never
answered, and the Pi could not ask or command anything.

**After**, the link is two-way. The Pi can still send its sensor numbers,
unchanged, but it can also send **commands** (`C,39,MOVE,12,...`), and the
brain **answers** (`@A,39`, then `@D,39,ok,traveled=12.01;...`).

```
Before:  Pi ──O,… / R,…──▶ brain                         (brain never answers)
After:   Pi ──O,… / R,…──▶ brain                         (same as before)
         Pi ──C,39,MOVE,12──▶ brain ──@A,39 … @D,39,ok,…──▶ Pi
```

| | Before | After |
|---|---|---|
| Who talks | Only the Pi | Both: the Pi asks, the brain answers |
| Can the Pi move the robot? | No | Yes, only in driver control, with STOP, joystick takeover and a 1 s cut-off |
| Sensor packets (`O`, `R`, `N`) | Read by the brain | Same; `vexpi` and `red_tracker` are unchanged |
| Who reads `stdin` on the brain | Anyone | **Only this module** |
| Brain's `stdout` | PROS COBS-encoded streams | Plain text lines the Pi can read |
| If the Pi breaks | Nothing happens | Still nothing: Pi motion stops, the robot keeps working |

## How the pieces fit: three layers

The brain side is three layers. Each one has one job and only talks to the
layer next to it.

| Layer | File | Job | Knows about the robot? |
| --- | --- | --- | --- |
| **`Link`** | `protocol.hpp` / `src/aon/pi/protocol.cpp` | The **post office**: reads and writes text lines, checks checksums, decides what each line is, queues motions (one at a time), answers, sends heartbeats, and runs the 1 s cut-off | No. No PROS, no motors. |
| **Commands** | `commands.hpp` / `src/aon/pi/commands.cpp` | The **list of verbs** (PING, STATUS, SENSORS, STOP, MOVE, TURN, PROBE, RESET_ODOM): checks arguments and limits, decides what each reply contains | Only through the abstract `Robot` interface |
| **Robot glue** | `pi-link.hpp` / `src/aon/pi/pi-link.cpp` | The **real robot**: `OverrideRobot` implements `Robot` with Override's `Drivetrain`, `Odometry`, motors and sensors. `start()` builds everything and starts the three PROS tasks. | Yes |

- **You don't edit the `Link`.** It is a power strip: you plug things into its
  sockets (`registerSensor`, `registerCommand`, `registerPacket`) from the
  other two files. Adding a sensor or a verb never means opening `protocol.cpp`.
- **`start()` in `pi-link.cpp` is the entry point, not the `Link`.** It creates
  the `Link` and the `OverrideRobot`, registers everything, and starts the three
  tasks:
  - **reader:** `stdin`, one byte at a time, into `Link::feed`
  - **worker:** runs the queued motion
  - **ticker:** heartbeat and cut-off, every 20 ms
- **The cable itself is touched only in `pi-link.cpp`:** the reader task reads
  `stdin`, and `writeLine` writes `stdout`. The `Link` only sees text.
- **Why `Link` and commands stay PROS-free:** the RaspberryPi repo compiles
  `protocol.cpp` and `commands.cpp` on a laptop for its brain simulator, so the
  end-to-end tests run the same code that ships. Never include PROS headers in
  those two files.

## How a command flows through the classes

A verb (an action the Pi can ask for) is connected to the `Link` in two
moments: once at startup, then every time that command arrives.

### Moment 1: at startup, the verb is pinned on the `Link`'s board

```
start()                                        pi-link.cpp
 └─ registerStandardCommands(*piLink, *robot)  commands.cpp
     └─ link.registerCommand("MOVE", Kind::MOTION, 1, 2, recipe)
         └─ commands.push_back({...})          protocol.cpp, Link::registerCommand
```

`registerCommand` is the socket. It stores five things in the `Link`'s list of
commands: the word, the kind (immediate or motion), the minimum and maximum
number of arguments, and the recipe (a lambda). Nothing runs yet.

### Moment 2: when the command arrives, the `Link` looks it up and runs it

```
"C,39,MOVE,12,180*62" arrives on stdin
 └─ Link::handleLine
     ├─ checks the checksum, reads seq = 39
     ├─ find("MOVE") on the board           not found → @D,39,err,unknown_verb
     ├─ counts the arguments (1 to 2?)      wrong     → @D,39,err,bad_args
     │
     ├─ IMMEDIATE (PING, STATUS, SENSORS, STOP):
     │    runs the recipe now and sends @D,39,...
     │
     └─ MOTION (MOVE, TURN, PROBE, RESET_ODOM):
          queues it (only one at a time; a second one gets err,busy)
          sends @A,39 ("got it")
              ⋮  the worker task picks it up
          Link::runPendingMotion
           ├─ runs the recipe (it blocks while the robot moves)
           └─ sends @D,39,ok,... (or aborted / timeout)
```

### Who does what

| Your recipe (in `commands.cpp`) | The `Link` (you never edit it) |
| --- | --- |
| Checks the **values** ("is 12 a valid distance?", limits) | Checks the **checksum** and the **number** of arguments |
| Tells the robot what to do, through `Robot` | Decides **when** it runs: now, or queued for the worker |
| Returns a `Result` (`ok` / `err` / `timeout` + fields) | Turns that `Result` into the `@D` line (split into `@P` lines if long) |
| | Handles busy, STOP, joystick takeover and the 1 s cut-off |

The only thing you write is the recipe; the `Link` handles everything around it.

### Where the hardware comes from

`commands.cpp` only knows the `Robot` interface, so it also runs in the
RaspberryPi simulator. If a new verb needs hardware that `Robot` doesn't have
yet (the intake, a piston...), choose one of two options:

| Option | How | Trade-off |
| --- | --- | --- |
| **A. Add a button to `Robot`** | Declare `virtual void intake(double pct) = 0;` in `commands.hpp`, implement it in `OverrideRobot` (`pi-link.cpp`) and in `SimRobot` (RaspberryPi `sim/brain_sim.cpp`), register the verb in `commands.cpp` | Testable in the simulator; three files |
| **B. Register it in `pi-link.cpp`** | Call `link.registerCommand("INTAKE", ...)` inside `start()`, using Override's objects directly | One file, but it only exists on the real robot; the simulator can't test it |

The socket is the same either way (`registerCommand`); only the file changes.

## What changes when you program the robot

Autons, driver control, PID and the motion functions work exactly as before.
There are three new rules:

1. **Don't read `stdin` yourself.** This module is the only reader. Two readers
   split the bytes between them and both break. Get Pi data from here instead,
   e.g. `aon::pi::latestOtosPose()`.
2. **New motion loops should check `abortRequested`**, like `driveProfiled` does
   (`while (... && !abortRequested)`). That is what lets STOP, the joystick and
   the cut-off stop them. A loop without it keeps running until it finishes on
   its own.
3. **Don't start your own `printf` lines with `@`.** `@` marks the brain's
   answers to the Pi. Any other output is fine; the Pi keeps it as "brain
   console".

## Hooks in existing Override files (already done)

The files in this folder are a new module. These are the small changes inside
Override's original files that plug it in: the "doors" between the new room
and the existing house. Line numbers drift as code changes, so search for the
symbol.

| # | File | Look for | What it does |
| --- | --- | --- | --- |
| 1 | `src/main.cpp` | `aon::pi::start(` in `initialize()` | Turns the link on at power-up |
| 2 | `src/main.cpp` | `aon::pi::hasControl()` in `opcontrol()` | Gate: while the Pi drives, skip the driver code and only call `checkDriverOverride()` (joystick takes control back) |
| 3 | `include/aon/globals.hpp` | `pi::abort("x_button")` in `aon::STOP()` | The X button (`autonSafety`) also stops Pi motion; it does nothing if the Pi isn't driving |
| 4 | `include/aon/drivetrain/drivetrain.hpp` | `abortRequested`, `requestAbort()`, `clearAbort()`, `isAbortRequested()` | The brake flag the Pi link raises and lowers |
| 5 | `include/aon/drivetrain/drivetrain.hpp` | `&& !abortRequested` in every motion loop; `settle \|\| abortRequested` before `stop()` | How each motion obeys the brake (`drivePID`, `turnPID`, `driveProfiled`, `strafeProfiled`, `turnProfiled`, `driveAngleOfArc`, `follow`) |
| 6 | `src/aon/drivetrain/*.cpp` | `&& !abortRequested` in `goToPose` / `follow` | The same brake check in each drivetrain type (differential, H, X, mecanum) |
| 7 | `include/aon/drivetrain/drivetrain.hpp` | `motorGroups()`, `getMaxVelocities()`, `setMaxVelocities()` | Let the link read the drive motors and apply the Pi's speed limit (restored after each motion) |
| 8 | `differential-drive.hpp`, `h-drive.hpp` | `motorGroups() override` | Name their motor groups `L`, `R` (and `M`) for the `motors` sensor |
| 9 | `include/aon/controls/s-curve-profile.hpp` | `getMaxVelocity()` | Read a profile's speed cap so it can be restored |
| 10 | `src/aon/odometry/odometry.cpp` | the "+/-180 seam" comment in `update()` | Bug fix: crossing ±180° made heading jump ~360° for one update and corrupted X/Y on arcs |
| 11 | `src/aon/odometry/odometry.cpp` | `trackingDistances()` | How far each tracking wheel has rolled, as odometry sees it (signed, in inches) |

### The brake flag, step by step

```
STOP / joystick / X button / 1 s cut-off
   └─ drivetrain.requestAbort()          abortRequested = true
        └─ the running loop (e.g. driveProfiled) checks it every lap (~20 ms)
             └─ leaves the loop and calls stop()
when the Pi's motion is over
   └─ drivetrain.clearAbort()            abortRequested = false
```

The `Link` raises the flag (through the `abortMotion` hook) and always lowers
it again when a Pi motion ends. So the flag is always down when an auton or
driver code runs: **autons are never cut short by a stale Pi abort.** It is
`std::atomic`, so the link's tasks and the motion loop can read and write it
at the same time safely.

**New motion code:** add `&& !abortRequested` to the loop condition and stop
the motors when it is set. Otherwise STOP and the cut-off can't interrupt it;
it would run until it finishes on its own.

## The Pi is an add-on

- With no Pi attached nothing changes: no control, no heartbeat output.
- The Pi can only move the robot in driver control. Motion is refused while
  disabled, in autonomous, or while the IMU is calibrating.
- A joystick move, the X button, a `STOP`, or 1 s without a command from the Pi
  aborts the motion and hands control back to the driver.
- **COBS:** `start()` switches stdout from COBS-framed streams to plain lines
  so the Pi can read them. If `pros terminal` shows odd characters, use
  `pros terminal --raw`.

### During autonomous: no movement, information still flows

| What | During autonomous |
| --- | --- |
| Motion verbs from the Pi (MOVE, TURN, PROBE, RESET_ODOM) | Refused: `err,code=refused` ("autonomous is running") |
| Questions from the Pi (STATUS, SENSORS, PING) | Answered |
| Sensor packets from the Pi to the brain (`O`, `R`, `N`, any registered tag) | Always accepted, in every mode |

Why: `opcontrol()` steps aside while the Pi drives (`hasControl()`), but
`autonomous()` has no such gate. Letting the Pi move the robot during auton
would put two drivers on the same motors. The Pi helps autonomous the other
way around: the auton stays the driver and **reads** the Pi's data, e.g.

```cpp
Pose otos; std::uint32_t age;
if (aon::pi::latestOtosPose(otos, age) && age < aon::pi::OTOS_TIMEOUT_MS) {
  // correct the path with the OTOS position
}
```

The LLM itself should never be in the auton loop: it takes seconds per
decision. The bridge is a debugging tool.

## Expanding it: the three sockets

Every extension plugs into one of three sockets. None of them touches the
`Link` itself.

| Socket | You want to... | Brain side | Pi side |
| --- | --- | --- | --- |
| **1. Sensor** | let the Pi **read** something | `link.registerSensor(...)` in `registerSensors()`, `pi-link.cpp` | **Nothing**: it appears in `read_sensors` by itself |
| **2. Command** | let the Pi **do** something | `link.registerCommand(...)` in `commands.cpp` | A tool in `bridge/server/tools/` |
| **3. Packet** | receive values from a sensor on the **Pi** | `link.registerPacket('X')` in `commands.cpp` | The `vexpi` runtime sends `X,...` lines |

### 1. A sensor on the brain (one step)

For a V5 sensor plugged into one of the brain's smart ports. In
`registerSensors()` in `src/aon/pi/pi-link.cpp` (there is a commented example
at the end of that function):

```cpp
// constants.hpp, in the right robot's section:  #define FRONT_DISTANCE_PORT 21
static pros::Distance frontDistance(FRONT_DISTANCE_PORT);
link.registerSensor("front_distance", [](KV &kv) {
  kv.add("mm", static_cast<int>(frontDistance.get_distance()));
});
```

The function is a recipe: it runs every time the Pi asks, so the values are
always fresh. Nothing is stored. The Pi needs no change; the LLM can now call
`read_sensors name=front_distance`.

#### Ports: keep each number in one place

The port number in the code must match the hole the cable is plugged into
(1-21). Today the robot's ports are written directly in `globals.hpp`, e.g.
`aon::Odometry(19, -18, 5, 0, 16)`.

- **The sensors already registered here never repeat a port number.** They
  ask the objects built in `globals.hpp` (`get_port()`, `get_port_all()`), so
  they always match the wiring.
- **A new sensor needs its own handle in `pi-link.cpp`**, because this file
  can't include `globals.hpp` (only `main.cpp` does). Two handles on the same
  port read the same sensor; a PROS sensor object is just a handle to a port.
- **So give the port a name** in `constants.hpp`, inside the right robot's
  section (`USING_BIG_ROBOT` or not), and use that name both in `globals.hpp`
  and here. Moving the cable to another hole is then a one-number change, and
  the two places can't disagree.
- **Never type the number directly.** For example, 5 is already the small
  robot's back tracking wheel and 8 its intake. Port 21 is the only one free
  on both robots today.

To check what is plugged where:
- the brain screen's **Devices** menu
- `diagnose` from the chat, which names missing hardware with its port, e.g.
  "left tracking rotation sensor (port 19) is not detected"

#### Which sensors to register

Only the ones on the robot that you would want to look at while debugging.
Registering is cheap, but every sensor makes every `read_sensors` and
`diagnose` reply longer, and one with nothing plugged in only adds "not
installed" noise.
- **Remove or comment out** the registration when the sensor leaves the robot.
- **While developing a new sensor**, registering it early is useful: you can
  read its values from the chat to check that it works. With nothing plugged in
  it reads as no value (`nan` / not installed); nothing breaks.

### 2. A command (brain + Pi)

**Brain**, in `registerStandardCommands()` in `src/aon/pi/commands.cpp`:
teach it the verb. `Kind::MOTION` gives it the one-at-a-time queue, STOP and
the cut-off; use `Kind::IMMEDIATE` for quick reads.

```cpp
link.registerCommand("INTAKE", Kind::MOTION, 1, 1, [&robot](const Args &args) {
  double speedPct = 0;
  if (!args.number(0, speedPct)) return Result::error("bad_args", "speed must be a number");
  robot.intake(speedPct);
  return Result::ok(KV().add("speed", speedPct));
});
```

`robot.intake()` doesn't exist yet: it is the new hardware this verb needs.
See [Where the hardware comes from](#where-the-hardware-comes-from) for the two
ways to add it (option A is shown here).

**Pi**, in a module under `bridge/server/tools/`: give the LLM a button for
it. The LLM finds new tools on its own through `GET /tools`.

```python
@tool("intake", "Spin the intake. Positive is in.",
      {"speed_pct": {"type": "number", "description": "Percent, -100 to 100"}},
      ["speed_pct"], motion=True)
async def intake(speed_pct: float) -> dict:
    msg = await ctx.link.request("INTAKE", speed_pct, timeout=5, motion=True)
    return ok(**msg.fields) if msg.status == "ok" else brain_failure(msg)
```

### 3. A sensor on the Pi, values going to the brain

For a sensor plugged into the **Raspberry Pi** (I²C, USB, GPIO), not the brain.
It uses no brain port: its data rides on the USB cable, exactly like OTOS.

```
sensor ──(I²C / USB)──▶ program on the Pi ──"D,412\n"──▶ USB cable ──▶ Link mailbox ──▶ your code
```

1. **Pi:** read the sensor and send one line per reading, starting with its tag:
   `D,<value>`. Add it to the `vexpi` runtime with its shared `PacketSender`
   (RaspberryPi `apps/main.cpp`; see that repo's README). Don't start a separate
   program that opens the port.
2. **Brain:** declare the tag, in `registerStandardCommands()` in
   `src/aon/pi/commands.cpp`:
   ```cpp
   link.registerPacket('D');
   ```
3. **Your code** (auton, driver, odometry), from any task:
   ```cpp
   std::vector<std::string> fields; std::uint32_t age;
   if (aon::pi::latestPacket('D', fields, age) && age < 300) {
     double mm = std::atof(fields[0].c_str());
   }
   ```
   Only the newest packet is kept. Decide yourself when a value is too old: if
   the Pi stops, the age keeps growing.
4. **Optional:** register a sensor that shows it to the Pi and the LLM, like
   `pi_otos` in `commands.cpp`.

Packets are accepted in every mode, autonomous included. Tags are one capital
letter: `C` is taken by commands, and `O`, `R` and `N` are in use.

### More sensors, more to debug?

Not much more. Each addition is a small, separate block: if one breaks, only
that one reports an error, and the rest keep working.

| If you want... | Add... |
| --- | --- |
| The Pi to **read** the new sensor | Only socket 1 |
| `diagnose` to **judge** it ("too close", "disconnected") | A check in RaspberryPi `bridge/server/tools/diagnostics.py`; the `analyze_*` functions are just numbers in, verdicts out |
| To **test** it without the robot | The same sensor in `sim/brain_sim.cpp` (a fake value), plus one test in `sim/test_pipeline.py` |

## Sensor packets from the Pi (OTOS, red target)

The Pi's other programs write to the same USB port as the bridge server:
- `vexpi` streams `O,<x>,<y>,<heading>` at 50 Hz.
- `red_tracker` sends `R,<in>` / `N,0`.

The reader task here is the **only** `stdin` reader on the brain, so those
packets reach Override through this module:

| Use | How |
| --- | --- |
| OTOS pose in odometry fusion | `aon::pi::latestOtosPose(pose, ageMs)`. Treat it as stale after `aon::pi::OTOS_TIMEOUT_MS` (300 ms). |
| Any packet, by tag | `link.registerPacket('X')` in `commands.cpp`, then `aon::pi::latestPacket('X', fields, ageMs)` from any task |
| From the Pi / the LLM | the `pi_otos` and `pi_target` sensors (`read_sensors`) |

Sensor packets are never answered, and they never keep the cut-off alive.
Only bridge commands do, so a dead bridge server still aborts the Pi's
motion while `vexpi` keeps streaming.

> **Merging with `main` (PR #29, Optical Tracking Odometry Sensor):** `main`
> now reads OTOS with its own `fgetc(stdin)` loop in `Odometry::initialize()`.
> With this module running, two readers would split the bytes between them
> and both would break. When the branches merge, replace that loop with
> `aon::pi::latestOtosPose()`; it gives the same pose and age the loop
> produced.
