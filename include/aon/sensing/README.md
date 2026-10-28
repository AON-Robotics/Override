# Sensing

This directory currently reserves a location for sensor-specific processing;
it contains no sensor implementation. Active odometry and pose-estimation code
lives in [../odometry](../odometry), with implementations in
[src/aon/odometry](../../../src/aon/odometry).

The current odometry modules include sensor measurements, pose estimation,
heading fusion and the EKF. Robot sensor configuration is assembled in
[globals.hpp](../globals.hpp); [src/main.cpp](../../../src/main.cpp) starts the
odometry task through the drivetrain.

Path execution requires available, fresh, finite odometry. Missing or stale
feedback rejects safely; freshly timestamped frozen or inaccurate feedback is
not proof of physical motion and may require timeout. Legacy drivetrain pose
getters/setters and initialization still require configured odometry. See the
[motion execution contract](../competition/README.md) for failure handling.

Sensor calibration, sensor truth, loaded target scheduling and physical tracking
remain hardware-validation work. Buffered path samples contain observed poses
and requested commands, not independent ground-truth position or measured RPM.
