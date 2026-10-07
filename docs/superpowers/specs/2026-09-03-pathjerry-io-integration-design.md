# PATH.JERRYIO Integration Design

## Goal

Integrate PATH.JERRYIO-generated paths into Override without a LemLib runtime dependency. The robot will parse exported path assets, preserve per-point speeds, and execute them using AON drivetrain, odometry, pure-pursuit, and motion-profile code.

## Constraints

- No LemLib headers, sources, types, or runtime calls.
- PATH.JERRYIO remains an offline path editor; AON owns decoding and execution.
- The integration must be reusable by any autonomous routine and must not encode one robot configuration or one match routine.
- The existing `Drivetrain::follow(std::vector<Pose>)` entry point remains source-compatible.
- Invalid input, timeout, disabled competition state, and empty paths must stop the drivetrain safely.
- PATH.JERRYIO point speeds are interpreted on its 0-127 scale and mapped to the active drivetrain's RPM limit.

## Architecture

### Path model and decoder

Add an AON path model containing ordered points with position and PATH.JERRYIO speed. A dedicated decoder accepts an embedded PROS asset or a bounded character buffer and parses the numeric data section up to `endData`. It ignores editor metadata after that terminator while validating finite coordinates, speed range, point count, and complete records. Decode errors are returned explicitly; malformed input never produces a partially usable path.

The supported export is the PATH.JERRYIO `LemLib v0.5` text layout because it is currently available in the editor and in the AON Push-Back Testing branch. This names the external file layout only; no LemLib code is used.

### AON path following

Add a speed-aware AON pure-pursuit controller with monotonic progress, geometric lookahead, remaining-distance calculation, and tank-drive output normalization. The current AON `MotionProfile` controls acceleration and deceleration while each target point's exported speed provides the local velocity ceiling. Heading calculations use the repository's VEX/odometry convention consistently.

The controller core is independent of PROS timing and hardware so it can be host-tested. The drivetrain execution layer owns timing, odometry sampling, motor commands, cancellation/disabled checks, timeout, terminal tolerance, final heading alignment, and guaranteed stop behavior.

### Public API

Expose AON-native APIs conceptually equivalent to the convenient LemLib autonomous calls:

- `PathJerryIO::decode(...)`
- `Drivetrain::followPath(path, options)`
- `Drivetrain::moveToPose(target, options)`
- existing `Drivetrain::turnToHeading(...)`

Options include lookahead distance, timeout, position tolerance, direction, maximum RPM, final-heading behavior, and control-loop period. Results distinguish completion, timeout, cancellation/disable, and invalid input.

### Assets and autonomous workflow

PATH.JERRYIO files live in `static/` using `<routine>-<leg>.jerryio.txt`. Autonomous code embeds each file with PROS `ASSET`, decodes it once, resets odometry to the documented starting pose, and calls the AON follower. A small example/validation routine demonstrates the workflow without binding the subsystem to a competition slot.

Add repository documentation covering editor configuration, coordinate units/convention, export format, naming, static asset embedding, decoding, following, and staged physical validation.

## Testing

Host tests cover valid decoding, metadata handling, malformed/truncated records, non-finite values, speed limits, monotonic follower progress, speed mapping, output normalization, completion, and reverse paths. A PROS build verifies firmware integration. Physical verification remains a staged robot procedure because host tests cannot validate odometry calibration or drivetrain tuning.

## Safety

All execution exits route through `Drivetrain::stop()`. Empty or invalid paths never command motors. The follower checks competition-disabled state every loop, enforces a positive timeout, normalizes commands to configured limits, and does not align final heading after cancellation or disable.
