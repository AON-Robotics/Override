#pragma once
// Hardware-only replacements shared by every host test; controller code is real.
#define _USE_MATH_DEFINES
#define _CRT_SECURE_NO_WARNINGS
#define _PROS_API_H_
#define _PROS_MISC_H_
#define _PROS_RTOS_HPP_
#define _PROS_SCREEN_HPP_
#define _PROS_MOTORS_HPP_
#include <algorithm>
#include <cassert>
#include <cmath>
#ifndef M_PI
#define M_PI 3.14159265358979323846
#define M_PI_2 1.57079632679489661923
#endif
#include <cstdint>
#include <functional>
#include <iostream>
#include <memory>
#include <vector>
namespace pros {
struct Mutex {
  bool take(unsigned) { return true; }
  void give() {}
};
struct Rotation {
  explicit Rotation(short port) : reversed(port < 0) {}
  bool reversed;
  double position = 0;
  double get_position() const { return reversed ? -position : position; }
  void set_position(double value) { position = reversed ? -value : value; }
  void reset() { position = 0; }
};
struct Imu {
  explicit Imu(short) {}
  double heading = 0;
  double get_heading() const { return heading; }
  double get_rotation() const { return heading; }
  void tare() { heading = 0; }
};
struct gps_position_s_t { double x = 0, y = 0; };
struct Gps {
  Gps(short, double, double, double, double, double) {}
  gps_position_s_t get_position() { return {}; }
};
inline std::uint32_t timeMs = 0, cancelAt = UINT32_MAX, disableAt = UINT32_MAX;
inline std::function<void(unsigned)> advance;
inline std::uint32_t millis() { return timeMs; }
inline std::uint64_t micros() { return static_cast<std::uint64_t>(timeMs)*1000; }
inline void delay(unsigned ms) { if (advance) advance(ms); timeMs += ms; }
inline void reset() { timeMs = 0; cancelAt = disableAt = UINT32_MAX; advance = nullptr; }
namespace competition { inline bool is_disabled() { return timeMs >= disableAt; } }
enum controller_id_e_t { E_CONTROLLER_MASTER };
enum { E_CONTROLLER_DIGITAL_B };
enum text_format_e_t { E_TEXT_LARGE_CENTER, E_TEXT_MEDIUM_CENTER, E_TEXT_SMALL, E_TEXT_MEDIUM, E_TEXT_LARGE };
struct screen_touch_status_s_t { int x = 0, y = 0, touch_status = 0; };
enum class Color { black, white, dark_gray, dark_red, dark_blue, dark_green, green, red, blue };
namespace c {
inline bool controller_get_digital(int, int) { return timeMs >= cancelAt; }
template<typename... T> void controller_print(int,int,int,const char*,T...) {}
}
namespace screen {
template<typename T> void set_eraser(T) {}
template<typename T> void set_pen(T) {}
inline void erase() {}
inline void erase_rect(int,int,int,int) {}
inline screen_touch_status_s_t touch_status() { return {}; }
template<typename... T> void print(int,int,const char*,T...) {}
template<typename... T> void print(int,int,int,const char*,T...) {}
}
enum class MotorBrake { coast, brake, hold };
enum class MotorGears { red, green, blue };
enum class MotorEncoderUnits { degrees };
struct Motor {
  explicit Motor(std::int8_t) {}
  std::int32_t move_velocity(std::int16_t) { return 1; }
  std::int32_t move_voltage(std::int16_t) { return 1; }
  void set_brake_mode(MotorBrake) {}
  void set_gearing(MotorGears) {}
  void set_encoder_units(MotorEncoderUnits) {}
  void tare_position() {}
  double get_actual_velocity() const { return 0; }
};
struct MotorGroup : Motor {
  explicit MotorGroup(const std::initializer_list<std::int8_t>&) : Motor(0) {}
};
namespace lcd {
template <typename... Args> void print(int, const char*, Args...) {}
}
}  // namespace pros

