# Path execution improvements

User-approved scope: diagnose straight-only runs, preserve export speeds, expose
per-path tuning, sequence actions, and make calibration/repeatability measurable.
Optimize new code for firmware size, bounded work, and readability.

## Design

Keep the existing AON controller and compatibility overloads. Add a non-owning
path view with optional byte speeds, configurable following, and callback-based
observations/actions. Keep conversion on the PC. Anchor a sequence once and use
views for its legs. Diagnostics are selectable separately from the match routine.
CSV recording is optional; missing storage must never prevent motion or stopping.
No new runtime dependencies, no changes to uncommitted hardware configuration.

## Execution

- [x] Test and implement compact speed-preserving conversion and path views.
- [x] Test configurable tolerances/lookahead, speed preview, curve limits, and
      steering with straight, curve, U-turn and rotated-start simulations.
- [x] Test runtime telemetry, settled completion, cancellation and progress hooks.
- [x] Replace copied legs with shared views; retain existing intake/piston order,
      add reusable sequences with bounded sensor waits and per-leg settings.
- [x] Add selectable diagnostic paths, optional CSV traces and run summaries;
      document calibration, clearance and repeated physical validation.
- [x] Run all host tests and ARM build; review changed code and measure size.

Physical tests are outstanding until run on the robot. Do not claim to have
identified the straight-only hardware failure from an ideal simulation.

Verified: 7 Python tests, 6 C++ host executables and ARM make -j4 pass.
Final hot image: 858092 bytes. text=746037, data=112008, bss=48237141.
Starting artifact text=750991, data=112008, bss=48237141; final code/data
is 4954 bytes smaller. This is an artifact comparison, not an isolated
benchmark of each optimization. Existing deprecation warnings remain.
