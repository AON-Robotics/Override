# OTOS odometry for autonomous

Override reads `O,<x inches>,<y inches>,<heading degrees>\n` from the V5 Brain's USB User Port through its odometry task. The RaspberryPi `otos_stream` app sends these packets at 50 Hz. Connect the Pi to the Brain's User Port (commonly `/dev/ttyACM1` on the Pi), and connect the OTOS to the Pi's I2C bus (commonly `/dev/i2c-1`).

On the Pi, build with `-DVEXPI_BUILD_VISION=OFF` if the camera SDK is unavailable, then run `otos_stream` before autonomous. Keep the robot still during startup IMU calibration. The app checks the OTOS, calibrates, resets tracking, and opens the serial port. It retries if the Brain is disconnected. Only one Pi program can own the User Port at a time; `red_tracker` and `otos_stream` cannot run concurrently on that port.

The Pi maps OTOS X right/Y forward/counterclockwise heading into Override X forward/Y right/clockwise heading. Override anchors the first received pose to `INITIAL_ODOMETRY_X`, `INITIAL_ODOMETRY_Y`, and `INITIAL_ODOMETRY_THETA`. A later `resetPose()` anchors the current sensor reading to the requested field pose. Pose data is considered stale after 300 ms; autonomous start and pose based motion stop when it is stale.

Before running on the field, measure the OTOS mount offset and linear/angular scalars in `apps/otos_stream.cpp` on the Pi. Confirm with the robot on blocks that pushing forward increases Override X, pushing right increases Y, and a clockwise turn increases heading. Test that disconnecting the Pi stops a pose based autonomous move. The existing camera target packets are ignored by the odometry reader.

## Debug GUI test

`TESTING_AUTONOMOUS` is enabled in `include/aon/constants.hpp`, which selects `GuiDebug` and leaves driver control inactive during this test mode. Open **DEBUG → DATA** to see `OTOS live` (1 for fresh packets, 0 for missing or stale), X, Y, and heading. **Live Graph** plots X against Y; **Field Map** records fresh pose samples only. The **Auton Runner** shows `OTOS OFFLINE` and `WAIT` until packets arrive, and it blocks RUN while the link is stale. Its RESET button reanchors the current sensor reading to the configured initial pose. Restore `TESTING_AUTONOMOUS` to `false` for regular operator control.

For incremental movement testing, select **DEBUG → Registered Autons → OTOS Square Builder**, then open **VARS**. The five steps are Move 1 (12 inches), Turn 1 (0 degrees), Move 2 (0 inches), Turn 2 (0 degrees), and Move 3 (0 inches). Adjust a value with the GUI buttons; a zero step is skipped. Return to **Auton Runner** and press **RUN**. The turns are relative, with positive degrees clockwise. Press **RESET** before a new run if you want the robot's current position to be field pose (0, 0, 0).
