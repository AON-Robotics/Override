# Review fixes implementation plan

Goal: apply Pablo's review while preserving the follower and sequencer.
Architecture: exports own segment metadata; generated routes carry indices and
headings. Odometry owns pose. Host tests share hardware stubs and execute real
production routines. Python discovers the host compiler on each platform.

- [x] Add generator regression fixtures with known segment endpoints, repeated
  positions, headings, and malformed metadata. Run them before implementing.
- [x] Generate endpoint indices/headings and use them in static-path.cpp; remove
  runtime coordinate matching, mutation, staticWaypointAt and staticPathAt.
- [x] Share PROS stubs and exercise real Drivetrain with a simulated subclass.
  Reproduce setter failures, then remove the redundant pose and forward setters
  to odometry without resetting sensors. Run actual diagnostics and actions.
- [x] Fold the basic U-turn into autonomous tests, inline guarded GUI selection,
  remove unused export/helper files and replace duplicated route documentation.
- [x] Replace the Windows-only runner with a Python runner supporting CXX,
  clang++, g++, and discovered MSVC; combine related C++ tests into fewer binaries.
- [x] Run Python/C++ tests and the ARM build, inspect the diff, and report limits.

Verification: 12 Python tests and the consolidated MSVC C++ host suite pass;
ARM make and GCC C++17 syntax checks pass. Scoped independent review resolved
the boundary-revisit defect. macOS/Linux runtime and physical robot trials were
not run. Generator requires ordered sampled boundaries and rejects ambiguous
earlier crossings; the older path export now includes its two missing boundaries.
