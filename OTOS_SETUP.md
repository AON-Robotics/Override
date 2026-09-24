# OTOS odometry for autonomous

Override reads `O,<x inches>,<y inches>,<heading degrees>\n` from the V5 Brain's USB User Port through its odometry task. The RaspberryPi `otos_stream` app sends these packets at 50 Hz. Connect the Pi to the Brain's User Port (commonly `/dev/ttyACM1` on the Pi), and connect the OTOS to the Pi's I2C bus (commonly `/dev/i2c-1`).

On the Pi, build with `-DVEXPI_BUILD_VISION=OFF` if the camera SDK is unavailable, then run `otos_stream` before autonomous. Keep the robot still during startup IMU calibration. The app checks the OTOS, calibrates, resets tracking, and opens the serial port. It retries if the Brain is disconnected. Only one Pi program can own the User Port at a time; `red_tracker` and `otos_stream` cannot run concurrently on that port.

The Pi maps OTOS X right/Y forward/counterclockwise heading into Override X forward/Y right/clockwise heading. Override anchors the first received pose to `INITIAL_ODOMETRY_X`, `INITIAL_ODOMETRY_Y`, and `INITIAL_ODOMETRY_THETA`. A later `resetPose()` anchors the current sensor reading to the requested field pose. Pose data is considered stale after 300 ms; autonomous start and pose based motion stop when it is stale.

Before running on the field, measure the OTOS mount offset and linear/angular scalars in `apps/otos_stream.cpp` on the Pi. Confirm with the robot on blocks that pushing forward increases Override X, pushing right increases Y, and a clockwise turn increases heading. Test that disconnecting the Pi stops a pose based autonomous move. The existing camera target packets are ignored by the odometry reader.

## Debug GUI test

`TESTING_AUTONOMOUS` is enabled in `include/aon/constants.hpp`, which selects `GuiDebug` and leaves driver control inactive during this test mode. Open **DEBUG → DATA** to see `OTOS live` (1 for fresh packets, 0 for missing or stale), X, Y, and heading. **Live Graph** plots X against Y; **Field Map** records fresh pose samples only. The **Auton Runner** shows `OTOS OFFLINE` and `WAIT` until packets arrive, and it blocks RUN while the link is stale. Its RESET button reanchors the current sensor reading to the configured initial pose. Restore `TESTING_AUTONOMOUS` to `false` for regular operator control.

For incremental movement testing, select **DEBUG → Registered Autons → OTOS Square Builder**, then open **VARS**. The five steps are Move 1 (12 inches), Turn 1 (0 degrees), Move 2 (0 inches), Turn 2 (0 degrees), and Move 3 (0 inches). Adjust a value with the GUI buttons; a zero step is skipped. Return to **Auton Runner** and press **RUN**. The turns are relative, with positive degrees clockwise. Press **RESET** before a new run if you want the robot's current position to be field pose (0, 0, 0).

## OTOS and V5 IMU heading

Override calibrates the V5 IMU at startup while the robot is still. Each valid OTOS packet updates X and Y from OTOS, while the V5 IMU supplies the heading used by autonomous. Its uneven steps are smoothed over about 0.02 seconds. OTOS heading is used only while the V5 IMU is unavailable or has an implausible jump. A returning IMU is aligned to the current pose without a jump. The first OTOS packet and every GUI RESET establish a new heading origin without moving the reported pose.

In **DEBUG → DATA**, compare `Heading (deg)` (V5 IMU based), `OTOS H (deg)` (unfiltered OTOS), and `IMU heading` (1 when the latest packet used the V5 IMU). Leave the robot still for a minute; `Heading (deg)` should remain near zero even if `OTOS H (deg)` creeps. Then turn it clockwise and check that both increase. The smoothing time is set by `V5_IMU_HEADING_SMOOTHING_SECONDS` in `include/aon/constants.hpp`; 0.02 seconds is a starting value. The V5 IMU can drift too, so compare both sensors while stationary. The Pi's OTOS angular scalar and mounting offset still need robot-specific calibration. OTOS X/Y are still the Pi's values; correcting the reported heading does not retroactively correct position error caused by OTOS heading drift.

`drivetrain.move()` now sends the same profiled RPM to both sides; it uses OTOS position to stop at the requested distance and does not steer from heading. Run the first move alone after **RESET** and compare `Heading (deg)` with `OTOS H (deg)` in **DEBUG → DATA**. If the physical robot turns while the heading readings stay near zero, check sensor mounting and IMU port. `turn()` and `goToPose()` still use heading feedback.
