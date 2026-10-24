# Tools

This is production helper and GUI code, included in the robot application.

- [vector.hpp](./vector.hpp): shared vector mathematics and geometry.
- [simple-filter.hpp](./simple-filter.hpp) and [moving-average.hpp](./moving-average.hpp): filtering helpers.
- [function-reader.hpp](./function-reader.hpp): registered function lookup and execution for autonomous routines and debug tools.
- [logging.hpp](./logging.hpp): terminal logging through stdout/stderr.
- [json.hpp](./json.hpp): bundled JSON implementation.
- [gui/](./gui/): robot-screen menus, autonomous selection, debug displays and graphs; see the [GUI guide](./gui/GUIDE_GUI.md).
- [gui-image-generator/](./gui-image-generator/): compiled GUI image data.

Path-controller updates perform no logging, formatting or file I/O. Optional
`Drivetrain::PathTrace` captures into caller-owned bounded storage and must be
read only after synchronous `followPath` returns. Samples distinguish requested
and converted RPM from physical measurements. Its complete contract is in the
[competition README](../competition/README.md).

As of 2026-10-24, motion software verification is complete within the documented
forward DifferentialDrive scope; hardware measurements and loaded target timing
remain outstanding. Host tests, simulation tools and audit records are local-only
development material. Root `/tools/` exclusions do not exclude this production
`include/aon/tools/` directory or `src/aon/tools/`.
