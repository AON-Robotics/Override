/**
 * @file simple-filter.hpp
 * @brief Compatibility wrapper for okapi::EKFFilter replacement in PROS4
 * @note This is a simple replacement for okapi::EKFFilter which uses a basic moving average filter
 * 
 * EKFFilter in PROS3 performed Extended Kalman Filtering for sensor fusion.
 * This replacement uses a simple exponential smoothing (low-pass filter) for basic noise reduction.
 */

#pragma once

#include <cmath>

namespace aon {

/**
 * @class SimpleFilter
 * @brief A simple low-pass filter to replace okapi::EKFFilter for PROS4
 * 
 * Uses exponential smoothing (exponential moving average) to filter noisy sensor readings.
 * This provides basic noise reduction similar to filtering, though not as sophisticated as EKF.
 */
class SimpleFilter {
  private:
    double alpha;  // Smoothing factor (0 < alpha <= 1, higher = more responsive)
    double filtered_value = 0.0;
    bool initialized = false;

  public:
    /**
     * @brief Constructs a SimpleFilter with a smoothing factor
     * @param alpha The smoothing factor (0 < alpha <= 1). Default 0.5
     * @param process_variance Unused, kept for API compatibility with okapi::EKFFilter
     */
    SimpleFilter(double alpha = 0.5, double process_variance = 0.0) 
        : alpha(std::max(0.01, std::min(1.0, alpha))), filtered_value(0.0), initialized(false) {}

    /**
     * @brief Filters a measurement value using exponential smoothing
     * @param measurement The raw measurement value
     * @return The filtered value
     */
    double filter(double measurement) {
      if (!initialized) {
        filtered_value = measurement;
        initialized = true;
      } else {
        // Exponential smoothing: y = alpha * x + (1 - alpha) * y_prev
        filtered_value = alpha * measurement + (1.0 - alpha) * filtered_value;
      }
      return filtered_value;
    }

    /**
     * @brief Resets the filter to uninitialized state
     */
    void reset() {
      initialized = false;
      filtered_value = 0.0;
    }

    /**
     * @brief Gets the current filtered value
     * @return The most recently filtered value
     */
    double getValue() const {
      return filtered_value;
    }
};

// Alias for compatibility
using EKFFilter = SimpleFilter;

}  // namespace aon
