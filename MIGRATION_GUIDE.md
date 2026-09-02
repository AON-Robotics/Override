# PROS 3 → PROS 4 Migration Guide

## Overview

This document details the complete migration of the AON robotics codebase from PROS 3 (OkapiLib) to PROS 4 (native APIs). The migration involved systematic replacement of deprecated OkapiLib APIs with PROS 4 native equivalents, refactoring of sensor fusion systems, and standardization of GUI color handling.

**Project Details:**
- **PROS Kernel:** Version 4.2.2 for V5 platform
- **Build System:** PROS-CLI 3.5.6 with CMake and Makefile
- **Language:** C++17
- **Optional Libraries:** LibLVGL 9.2.0 for display functionality
- **Completion Date:** September 2, 2026

---

## Key API Changes

### Motor Control

**OkapiLib (PROS 3):**
```cpp
okapi::Motor motor(port);
okapi::MotorGroup motorGroup({port1, port2});
motor.moveVelocity(rpm);
motor.moveVoltage(voltage);
motor.setBrakeMode(okapi::AbstractMotor::brakeMode);
```

**PROS 4 Native:**
```cpp
pros::Motor motor(port);
pros::MotorGroup motorGroup({port1, port2});
motor.move_velocity(rpm);
motor.move_voltage(voltage);
motor.set_brake_mode(pros::MotorBrake::hold);
```

**Changes:**
- `okapi::Motor` → `pros::Motor`
- `okapi::MotorGroup` → `pros::MotorGroup`
- Method naming: camelCase → snake_case
- Enum types: `okapi::AbstractMotor::brakeMode` → `pros::MotorBrake`
- Constructor: `okapi::Motor objects` → `int8_t port numbers` via `std::initializer_list<std::int8_t>`

### Pneumatics (ADI)

**OkapiLib (PROS 3):**
```cpp
pros::ADIDigitalOut piston(adi_port, false);
```

**PROS 4 Native:**
```cpp
pros::adi::DigitalOut piston(adi_port, false);
```

**Changes:**
- `pros::ADIDigitalOut` → `pros::adi::DigitalOut`
- `pros::ADIDigitalIn` → `pros::adi::DigitalIn`
- Namespace change: direct `pros::` → `pros::adi::` subfolder

### Rotation Sensor

**OkapiLib (PROS 3):**
```cpp
okapi::Rotation encoder(port, reversed);
```

**PROS 4 Native:**
```cpp
pros::Rotation encoder(port);
// Reversal must be handled manually if needed
```

**Changes:**
- Constructor: 2 parameters → 1 parameter (port only)
- Encoder reversal: Must implement manual reversal logic (negate values)
- No automatic reversal support in PROS 4 constructor

### GPS

**OkapiLib (PROS 3):**
```cpp
okapi::IMU imu(port);
auto status = imu.getStatus();  // Returns gps_status_s_t
```

**PROS 4 Native:**
```cpp
pros::Gps gps(port);
auto pos = gps.get_position();     // Returns pros::gps_position_s_t
auto heading = gps.get_heading();  // Individual method
auto velocity = gps.get_velocity(); // Individual method
```

**Changes:**
- Individual getter methods instead of status structs
- Type: `gps_status_s_t` → `pros::gps_position_s_t` (for position)
- All GPS data accessed via separate methods

### Logging

**OkapiLib (PROS 3):**
```cpp
static okapi::Logger logger;
logger.info("message");
```

**PROS 4 Native:**
```cpp
// Use standard C I/O
printf("message\n");
fprintf(stderr, "error\n");
```

**Changes:**
- Remove all `okapi::Logger` references
- Use standard `printf()` / `fprintf()` to stdout/stderr
- No built-in timestamp support (can be added manually)

### Filtering (EKFFilter)

**OkapiLib (PROS 3):**
```cpp
okapi::EKFFilter ekf;
```

**PROS 4 Alternative:**
```cpp
// Simple exponential smoothing wrapper (new file: simple-filter.hpp)
aon::SimpleFilter filter(alpha);  // alpha: 0-1 smoothing factor
double filtered = filter.filter(raw_value);

// For sophisticated systems, implement custom Extended Kalman Filter
// See include/aon/odometry/ekf.hpp for complete EKF implementation
```

**Changes:**
- No built-in EKF in PROS 4
- Created `SimpleFilter` class for basic low-pass filtering
- Full custom EKF implementation available for advanced localization

---

## Files Modified

### Core System Files

#### `include/main.hpp`
**Purpose:** Main entry point and global includes

**Changes:**
- ✅ Removed `#include "okapi/api.hpp"`
- ✅ Now includes only `#include "api.h"` (PROS 4)
- ✅ All project headers include via `#include "aon/api.hpp"`

---

### Motor Control

#### `include/aon/controls/smart_motor.hpp`
**Purpose:** Motor wrapper class with acceleration control and slew rate limiting

**Changes:**
- ✅ Changed inheritance: `okapi::Motor` → `pros::Motor`
- ✅ Constructor: `(const std::initializer_list<okapi::Motor>&)` → `(const std::initializer_list<std::int8_t>&)`
- ✅ Method names: camelCase → snake_case
  - `moveVelocity()` → `move_velocity()`
  - `moveVoltage()` → `move_voltage()`
  - `setBrakeMode()` → `set_brake_mode()`
- ✅ Enum type: `okapi::AbstractMotor::brakeMode` → `pros::MotorBrake`

**Status:** ✅ Complete

#### `include/aon/drivetrain/drivetrain.hpp`
**Purpose:** Abstract base class for all drivetrain implementations

**Changes:**
- ✅ Updated enum types in virtual methods
  - `okapi::AbstractMotor::brakeMode` → `pros::MotorBrake`
  - `okapi::motorGearset::gearset` → `pros::MotorGears`
  - `okapi::AbstractMotor::encoderUnits` → `pros::MotorEncoderUnits`
- ✅ Virtual method signature: `virtual void setBrakeMode(pros::MotorBrake) = 0;`

**Status:** ✅ Complete

#### `include/aon/drivetrain/{differential,h,mecanum,x}-drive.hpp`
**Purpose:** Specific drivetrain implementations

**Changes:**
- ✅ Constructor signatures: `(const std::initializer_list<okapi::Motor>&)` → `(const std::initializer_list<std::int8_t>&)`
- ✅ Updated virtual method overrides to use PROS 4 enum types
- ✅ Method names converted to snake_case

**Status:** ✅ Complete

**Files:**
- `differential-drive.hpp`
- `h-drive.hpp`
- `mecanum.hpp`
- `x-drive.hpp`

#### `src/aon/drivetrain/` (Implementation files)
**Status:** ✅ Complete - Constructors and methods updated per header changes

---

### Game Mechanisms

#### `include/aon/intake/intake.hpp` & `src/aon/intake.cpp`
**Purpose:** Game-specific intake system control

**Changes:**
- ✅ Motor group: `okapi::MotorGroup` → `pros::MotorGroup`
- ✅ Member: `std::shared_ptr<void> leverController = nullptr;`
- ⚠️ TODO: Replace with PROS 4 AsyncPositionController equivalent

**Status:** ⚠️ Partial - Lever control placeholder pending implementation

**Implementation Notes:**
```cpp
// Currently disabled with placeholder implementation
void extendLever() { 
  // TODO: Implement async position control
}
void resetLever() { 
  // TODO: Implement async position control  
}
bool leverFinished() { return true; }
```

#### `include/aon/piston/piston.hpp` & `src/aon/piston.cpp`
**Purpose:** Pneumatic control via ADI

**Changes:**
- ✅ `pros::ADIDigitalOut` → `pros::adi::DigitalOut`
- ✅ Constructor and method calls updated

**Status:** ✅ Complete

---

### Odometry & Localization

#### `include/aon/odometry/odometry.hpp`
**Purpose:** Odometry system combining wheel encoders and gyroscope

**Changes:**
- ✅ Removed OkapiLib includes
- ✅ Rotation sensor: 2-param constructor → 1-param (port only)
- ⚠️ TODO: Manual encoder reversal needed
- ⚠️ GPS integration simplified to individual getter methods

**Status:** ⚠️ Partial - Encoder reversal needs implementation

#### `src/aon/odometry.cpp`
**Purpose:** Odometry calculation implementation

**Changes:**
- ✅ Constructor updated for PROS 4 `pros::Rotation`
- ✅ GPS methods: `gps.get_position()`, `gps.get_heading()`, `gps.get_velocity()`
- ✅ Removed `gps_status_s_t` dependency
- ⚠️ TODO: "Handle encoder reversal for PROS4 - currently not supported in constructor"

**Code Example:**
```cpp
// Old: pros::Rotation encoder(port, reversed)
// New: Manual reversal required
pros::Rotation encoder(port);
// In update: value = encoder.get_angle();
// For reversed: value = -encoder.get_angle();
```

**Status:** ⚠️ Partial - Encoder reversal pending

#### `include/aon/odometry/ekf.hpp` (NEW)
**Purpose:** Extended Kalman Filter for sophisticated localization

**Status:** ✅ Complete

**Key Classes:**
- `struct EkfConfig` - Configuration parameters
- `class ExtendedKalmanFilter` - Full Kalman filtering implementation
- Key methods: `reset()`, `predict()`, `updateImuHeading()`, `updateGpsPosition()`, `pose()`

#### `src/aon/odometry/ekf.cpp` (NEW)
**Purpose:** EKF implementation with full Kalman filtering mathematics

**Changes:**
- ✅ Moved from `src/aon/tools/ekf.cpp` to `src/aon/odometry/ekf.cpp`
- ✅ Include path: `#include "../../include/aon/odometry/ekf.hpp"`
- ✅ Complete matrix operations and covariance handling

**Status:** ✅ Complete

#### `include/aon/odometry/pose-estimator.hpp` (NEW)
**Purpose:** Pose estimation data structures and motion calculation

**Status:** ✅ Complete

**Key Types:**
- `struct EstimatorPose` - Position (x, y, heading)
- `struct LocalMotion` - Robot-local motion deltas
- `struct TrackingGeometry` - Wheel configuration

#### `src/aon/odometry/pose-estimator.cpp` (NEW)
**Purpose:** Motion calculation and pose propagation

**Status:** ✅ Complete

**Key Functions:**
- `LocalMotion localMotion(WheelDeltas, TrackingGeometry)` - Convert wheel deltas to robot-local motion
- `EstimatorPose propagatePose(EstimatorPose, LocalMotion)` - Propagate pose forward

#### `include/aon/odometry/sensor-measurements.hpp` (NEW)
**Purpose:** Sensor input data structures and validation

**Status:** ✅ Complete

**Key Classes:**
- `class GpsGate` - Validates GPS measurements
- `class GpsFreshnessTracker` - Detects stale/duplicate samples

#### `include/aon/tools/simple-filter.hpp` (NEW)
**Purpose:** Exponential smoothing filter to replace OkapiLib's EKFFilter

**Status:** ✅ Complete

**Implementation:**
```cpp
class SimpleFilter {
  double alpha;  // Smoothing factor (0-1)
  double state;
  
  double filter(double measurement);
};

// Backward compatibility alias
using EKFFilter = SimpleFilter;
```

---

### Vision & Sensing

#### `include/aon/proximity/proximity.hpp` & `src/aon/proximity.cpp`
**Purpose:** Proximity sensor input handling

**Status:** ✅ Complete

#### `include/aon/orbit/orbit.hpp` & `src/aon/orbit.cpp`
**Purpose:** Vision-based orbit/rotation control

**Changes:**
- ✅ Rotation sensor: 2-param → 1-param constructor
- ✅ Filter: `okapi::EKFFilter` → `aon::EKFFilter`
- ⚠️ TODO: "Handle encoder reversal for PROS4"

**Status:** ⚠️ Partial - Encoder reversal pending

---

### GUI & Logging

#### `include/aon/tools/gui/ui/gui-layout.hpp`
**Purpose:** UI Button class and predefined button instances

**Changes:**
- ✅ **Button struct refactoring:**
  - Changed: `std::uint32_t bg, fg;` → `pros::Color bg, fg;`
  - Added constructor for assignment syntax support
  - Type conversions centralized in `draw()` method
- ✅ Removed `static_cast<uint32_t>()` from all button initializations
- ✅ Button instances now use clean syntax: `{x1, y1, x2, y2, "LABEL", pros::Color::red, pros::Color::white}`

**Before:**
```cpp
class Button {
  std::uint32_t bg, fg;
};
inline const ui::Button btn = {..., static_cast<uint32_t>(pros::Color::red), ...};
```

**After:**
```cpp
class Button {
  pros::Color bg, fg;
  
  Button(int x1=0, int y1=0, int x2=0, int y2=0, const std::string& label="",
         pros::Color bg=pros::Color::black, pros::Color fg=pros::Color::white,
         std::function<void()> onPress=nullptr)
      : x1(x1), y1(y1), x2(x2), y2(y2), label(label), bg(bg), fg(fg), onPress(onPress) {}
  
  void draw(int textFmt = pros::E_TEXT_MEDIUM) const {
    pros::screen::set_eraser(static_cast<uint32_t>(bg));  // Cast only here
    // ...
    pros::screen::set_pen(static_cast<uint32_t>(fg));     // Cast only here
    // ...
  }
};

inline const ui::Button btn = {..., pros::Color::red, pros::Color::white};
```

**Status:** ✅ Complete

#### `src/aon/tools/gui/ui/gui-displays.cpp`
**Purpose:** GUI display functions for main menu and autonomous selections

**Changes:**
- ✅ Button color assignments work with new `pros::Color` members
- ✅ Implicit conversion in assignment context: `btn.bg = pros::Color::red;`

**Status:** ✅ Complete (no changes needed after Button refactoring)

#### `src/aon/tools/gui/debug-tools/autonrunner.cpp`
**Purpose:** Autonomous routine selection and execution UI

**Changes:**
- ✅ Color standardization: Local `COLOR_*` macros → `pros::Color::*` enum
- ✅ Final pattern: `pros::screen::set_eraser(pros::Color::black);` (no static_cast)

**Status:** ✅ Complete

#### `src/aon/tools/gui/debug-tools/livegraph.cpp`
**Purpose:** Real-time data graphing UI

**Changes:**
- ✅ Color constants standardized to `pros::Color::*` enum usage

**Status:** ✅ Complete

#### `src/aon/tools/gui/debug-tools/variableadjuster.cpp`
**Purpose:** Runtime variable adjustment UI

**Changes:**
- ✅ Button array assignments removed `static_cast<uint32_t>()`
- ✅ Clean syntax: `btns[0] = {..., pros::Color::dark_red, pros::Color::white, ...};`

**Status:** ✅ Complete

#### `src/aon/tools/gui/debug-tools/fieldmapper.cpp`
**Purpose:** Field position tracking and analysis UI

**Changes:**
- ✅ Fixed ternary operator type mismatches
- ✅ Pattern: `uint32_t var = (condition) ? static_cast<uint32_t>(pros::Color::color) : CUSTOM_CONST;`
- ✅ Maintained custom color constants for UI: `COLOR_FIELD_BG`, `COLOR_GRID`, `COLOR_ORIGIN`, etc.

**Status:** ✅ Complete

#### `include/aon/tools/logging.hpp` & `src/aon/tools/logging.cpp`
**Purpose:** Centralized logging infrastructure

**Changes:**
- ✅ **Complete rewrite** from OkapiLib-based to standard C I/O
- ✅ All `okapi::Logger` references removed
- ✅ Implementation: `printf()` / `fprintf()` to stdout/stderr

**Old:**
```cpp
static okapi::Logger logger;
logger.info("message");
```

**New:**
```cpp
inline void log(const char* format, ...) {
  va_list args;
  va_start(args, format);
  vfprintf(stdout, format, args);
  va_end(args);
}
```

**Status:** ✅ Complete

#### `include/aon/tools/general.hpp`
**Purpose:** General utility functions

**Status:** ✅ Complete - OkapiLib includes removed

#### `include/aon/tools/json.hpp`
**Purpose:** JSON parsing and generation

**Status:** ✅ Complete - OkapiLib-independent

---

## Problem Resolution

### Issues Encountered & Solutions

| Issue | Root Cause | Solution |
|-------|-----------|----------|
| Missing OkapiLib includes | PROS 4 is complete rewrite; no compatibility layer | Systematically replaced all okapi:: with pros:: equivalents |
| Motor/MotorGroup class changes | API redesign in PROS 4 | Updated inheritance and method signatures |
| Enum type mismatches | PROS 4 uses top-level enums instead of nested | Replaced with `pros::MotorBrake`, `pros::MotorGears`, etc. |
| Method naming inconsistencies | OkapiLib: camelCase; PROS 4: snake_case | Global sed replacement for consistency |
| Constructor signature changes | PROS 4 motors take port numbers, not objects | Updated to `std::initializer_list<std::int8_t>` |
| Logger not available | OkapiLib logger removed in PROS 4 | Implemented printf/fprintf wrapper |
| EKFFilter unavailable | No built-in EKF in PROS 4 | Created SimpleFilter wrapper + custom EKF implementation |
| AsyncPositionController unavailable | Removed in PROS 4 | Created placeholder with TODO comments for custom implementation |
| Rotation sensor constructor | 2-param → 1-param; reversal removed | Added TODO for manual reversal logic |
| GPS status type deprecation | `gps_status_s_t` removed | Switched to individual getter methods |
| Color type mismatch in UI | `uint32_t` vs `pros::Color` enum | Refactored Button struct to accept `pros::Color` directly |
| Ternary operator type incompatibility | Mixed enum and uint32_t in conditional | Added explicit casts in ternary branches |
| Button initialization static_cast proliferation | Aggregate initialization can't implicitly convert enum | Added Button constructor + centralized casts in draw() method |

---

## Compilation Status

### ✅ Completed Migrations

- All source files compile without errors
- Motor control system fully functional
- Drivetrain implementations (differential, mecanum, x-drive, h-drive)
- Pneumatic control (pistons)
- Vision/orbit system (with encoder reversal TODO)
- Odometry system (with encoder reversal TODO)
- GUI system with properly typed Button class
- Logging infrastructure
- EKF-based localization system
- Sensor fusion pipeline

### ⚠️ Pending Implementations

1. **Encoder Reversal** (~20 lines per file)
   - Files: `src/aon/odometry.cpp`, `src/aon/orbit.cpp`
   - Implementation: Manual negation of encoder values when needed
   - Priority: Medium

2. **Async Position Control** (~50-100 lines)
   - File: `src/aon/intake.cpp`
   - Task: Replace placeholder lever control with state machine
   - Priority: Medium

3. **Performance Validation**
   - SmartMotor acceleration control runtime verification
   - Logger performance with printf/fprintf
   - GUI rendering performance with new Button class

---

## Testing Recommendations

### Unit Testing
```cpp
// Test motor control
pros::Motor motor(1);
motor.move_velocity(100);

// Test drivetrain
aon::MecanumDrive drive({1, 2, 3, 4});
drive.move(100, 100);

// Test GUI
ui::Button btn(10, 10, 100, 50, "TEST", pros::Color::red, pros::Color::white);
btn.draw();
```

### Integration Testing
- Test full drivetrain movement in all directions
- Verify odometry accuracy with wheel encoders
- Validate GPS positioning
- Test autonomous routines
- Verify GUI responsiveness

### Hardware Testing
- Upload to V5 brain
- Test all subsystems
- Validate sensor readings
- Profile runtime performance

---

## Build & Upload

### Build Project
```bash
cd /Users/pdieppa/Documents/GitHub/pros4/pros4\ migration
pros build
```

### Upload to Hardware
```bash
pros upload
```

### Build Flags
- C++17 support enabled
- LibLVGL 9.2.0 integration
- PROS 4.2.2 kernel

---

## Key Takeaways

1. **PROS 4 is a complete rewrite** - No OkapiLib compatibility layer exists; systematic replacement required
2. **Naming convention change** - camelCase (OkapiLib) → snake_case (PROS 4)
3. **Type safety matters** - Enum class vs uint32_t distinction important for aggregate initialization
4. **Centralize conversions** - Type casting at API boundaries cleaner than scattered throughout codebase
5. **Advanced features require custom implementation** - EKF, async position control, etc. must be built from scratch
6. **Plan for missing features** - Some PROS 3 conveniences (async controllers, automatic encoder reversal) need manual implementation in PROS 4

---

## References

- PROS 4.2.2 Documentation: https://pros.cs.purdue.edu/v4/
- PROS C++ API: https://docs.pros.rs/4.2.2/
- LibLVGL Documentation: https://docs.lvgl.io/v9/
- Project Location: `/Users/pdieppa/Documents/GitHub/pros4/pros4 migration`

---

**Document Version:** 1.0  
**Last Updated:** September 2, 2026  
**Status:** Migration Complete (pending encoder reversal and async position control implementations)
