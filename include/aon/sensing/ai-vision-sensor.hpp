#pragma once

#include "pros/ai_vision.hpp"

#include <cstdint>
#include <optional>
#include <vector>

namespace aon {

/// @brief what object to search for; used to configure enable or disable detection functions (ex: vision.enableDetection(aon::AIVisionMode::Tags))
enum class AIVisionMode { Tags, Colors, Objects, All };

/// @brief Identifies object found
enum class AIVisionDetectionType { Color, Code, Object, Tag };


/// @brief provides info about a target
struct AIVisionDetection {
  int id = 0; // iedntifier reported by sensor
  AIVisionDetectionType type = AIVisionDetectionType::Object; // kind of detection
  
  int x = 0; // top left corner of bounding box (x & y)
  int y = 0;
  int width = 0; // bounding box dimensions (W & height)
  int height = 0;
  double centerX = 0; // bound box center
  double centerY = 0;
  // AI model score (0-100); absent for colors, codes, and tags.
  std::optional<int> confidence; // value could b absent
};

/// @brief list of detections
struct AIVisionReadResult {
  std::vector<AIVisionDetection> detections;
  int error = 0;
  bool succeeded() const { return error == 0; }
};

/// @brief holds one optional target
struct AIVisionLookupResult {
  std::optional<AIVisionDetection> detection;
  int error = 0;
  bool succeeded() const { return error == 0; }
};
// both result functions check if there are errors or no targets found or no targets are matches



// CLASS
class AiVisionSensor {
 private:
  pros::AIVision sensor; // sensor is the object that communicates with the hardware
  int lastError = 0;

  std::optional<AIVisionDetection> convertObject(const pros::AIVision::Object& object) const; // converts raw PROS to aivisiondetection format

 public:
  explicit AiVisionSensor(std::uint8_t port);

  // Enable/disable modes without resetting stored sensor configuration.
  // Returns false on failure
  bool enableDetection(AIVisionMode mode);
  bool disableDetection(AIVisionMode mode);

  // Returns the enabled mode bitmask, or -1 on error.
  int getEnabledDetectionModes();

  // Select a tag family; replace=true clears previously enabled families.
  bool setTagFamily(pros::AivisionTagFamily family, bool replace = false);

  // Preferred autonomous API: data and error are returned together
  AIVisionReadResult readDetections();

  // Read once and return the matching detection with largest width * height.
  // Equal areas keep the first match in sensor order. Largest does not imply closest.
  AIVisionLookupResult readDetection(
      int id, std::optional<AIVisionDetectionType> type = std::nullopt);

  // Empty on no detections or failure; check getLastError() to distinguish.
  std::vector<AIVisionDetection> getDetections();

  // Returns the largest matching bounding box, or nullopt; ties keep the first.
  // Supply a type to distinguish identical IDs from different detection modes.
  std::optional<AIVisionDetection> getDetection(
      int id, std::optional<AIVisionDetectionType> type = std::nullopt);

  // Returns -1 on failure, zero when nothing is visible.
  int getDetectionCount();

  // Error from the last configuration/read operation; zero means success.
  int getLastError() const { return lastError; }
};

}  // namespace aon
