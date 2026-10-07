# JerryIO Speed, Action Markers, and GUI Option 4 Design

## Objective

Make AON's PATH.JERRYIO autonomous smoother and faster, support safe mechanism
actions at intentional path stops, and expose the routine without replacing any
existing Red or Blue autonomous slot.

## Diagnosed performance causes

The current routine caps the drivetrain at 150 RPM although the configured
drivetrain supports 600 RPM. The checked-in path's exported speeds average
approximately 57.5 on PATH.JERRYIO's 0-127 scale, yielding about 68 RPM average
at the current cap. The 171-inch route therefore takes roughly 12 seconds even
before physical losses. The path also contains a near-90-degree sample heading
change. The controller currently slews toward each local requested speed but
does not plan braking across future samples or limit velocity from curvature.

## Path-wide velocity planning

Complete the already-started AON follower interface in the working tree rather
than replacing it. During `PathFollower` construction, derive a velocity limit
for every waypoint from:

1. The preserved PATH.JERRYIO 0-127 speed converted through `maximumRpm`.
2. A backward braking pass so the robot can reach every later zero-speed point
   under `maximumDeceleration`.
3. An optional curvature limit using `maximumLateralAcceleration`, drive-wheel
   diameter, and motor-to-wheel ratio.

The existing per-update acceleration/deceleration slew remains the final command
rate limiter. The routine will use a conservative 350 RPM ceiling, a moderately
larger lookahead, and an explicitly configured lateral-acceleration limit. These
values remain tuning parameters and must be physically validated.

## Zero-speed action markers

An internal PATH.JERRYIO waypoint with speed zero is an action marker. The final
zero-speed waypoint remains only the terminal stop. Markers are numbered in path
order. Callers provide zero or more actions associated with marker ordinals.

Each action contains:

- the marker ordinal;
- a maximum/duration time in milliseconds;
- a start callback; and
- a cleanup callback.

The executor divides the path into motion legs at internal zero-speed markers.
For each marker it:

1. completes the preceding leg and stops the drivetrain;
2. invokes the start callback;
3. waits for the configured duration while polling competition-disable,
   cancellation, and the overall deadline;
4. always invokes cleanup if the action started; and
5. resumes the next leg with a safe nonzero launch command derived from the next
   exported sample.

Actions with zero duration support immediate piston state changes. A missing
action does not make a valid path fail: the marker is treated as a stop followed
by an immediate resume. An action referencing a nonexistent marker is invalid
configuration. Multiple actions may share a marker and execute in declaration
order. The routine's overall timeout includes both driving and action time.

The public API remains AON-owned and accepts callbacks without depending on the
Intake or Piston classes. The team routine demonstrates an intake action without
coupling the generic JerryIO subsystem to robot globals.

## Status and failures

The existing motion statuses, Brain display, controller message, and rumble are
retained. Invalid action configuration reports `InvalidOptions`. Disable,
cancellation, or overall timeout during an action stops the drivetrain, runs the
action cleanup callback, and returns the corresponding terminal status.

## GUI integration

Red and Blue autonomous arrays expand from three to four entries while Skills
remains at three. Existing options 1-3 are preserved, including restoring Red 1
to Black Beard. JerryIO Path (`JIO`) becomes both Red 4 and Blue 4. Selecting
either calls the same autonomous wrapper; the selected alliance still updates
`ALLIANCE`, allowing mechanism behavior such as color sorting to differ.

The Red and Blue selection screens use a 2x2 button grid with four distinct
touch targets. Skills retains its current three-button layout. Index validation
uses the selected list's actual size rather than a hard-coded maximum of three.
Red 4 is preselected on boot for testing.

## Documentation

Update the JerryIO README with velocity-planning parameters, zero-speed marker
semantics, action examples, cleanup behavior, and tuning guidance. Update the GUI
guide to document four Red/Blue entries, three Skills entries, and Red 4
preselection.

## Verification

Host tests must cover:

- braking before terminal and internal zero-speed points;
- curvature-based velocity limits;
- invalid physical/profile configuration;
- marker discovery and leg construction;
- ordered actions, immediate actions, timeout, disable/cancel, and cleanup;
- correct treatment of the final zero-speed point;
- Red/Blue option 4 index selection without changing Skills bounds; and
- the checked-in PATH.JERRYIO asset.

The complete PROS firmware must build from a clean state and contain the embedded
path symbol. Physical testing starts on blocks, then at reduced speed, before
using the 350 RPM ceiling.
