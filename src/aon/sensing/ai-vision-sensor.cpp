#include "aon/sensing/ai-vision-sensor.hpp"

#include "pros/error.h"

#include <algorithm>
#include <cerrno>

namespace aon {
namespace {
std::optional<pros::AivisionModeType> toProsMode(AIVisionMode mode) {
  switch (mode) {
    case AIVisionMode::Tags: return pros::AivisionModeType::tags;
    case AIVisionMode::Colors: return pros::AivisionModeType::colors;
    case AIVisionMode::Objects: return pros::AivisionModeType::objects;
    case AIVisionMode::All: return pros::AivisionModeType::all;
  }
  return std::nullopt;
}
}  // namespace

AiVisionSensor::AiVisionSensor(std::uint8_t port) : sensor(port) {}

int AiVisionSensor::getEnabledDetectionModes() {
  errno = 0;
  const auto modes = sensor.get_enabled_detection_types();
  lastError = modes == PROS_ERR ? (errno ? errno : EIO) : 0;
  return lastError ? -1 : modes;
}

bool AiVisionSensor::setTagFamily(pros::AivisionTagFamily family, bool replace) {
  errno = 0;
  const auto result = sensor.set_tag_family(family, replace);
  lastError = result == PROS_ERR ? (errno ? errno : EIO) : 0;
  return lastError == 0;
}

bool AiVisionSensor::enableDetection(AIVisionMode mode) {
  const auto mapped = toProsMode(mode);
  if (!mapped) {
    lastError = EINVAL;
    return false;
  }
  errno = 0;
  const auto result = sensor.enable_detection_types(*mapped);
  lastError = result == PROS_ERR ? (errno ? errno : EIO) : 0;
  return lastError == 0;
}

bool AiVisionSensor::disableDetection(AIVisionMode mode) {
  const auto mapped = toProsMode(mode);
  if (!mapped) {
    lastError = EINVAL;
    return false;
  }
  errno = 0;
  const auto result = sensor.disable_detection_types(*mapped);
  lastError = result == PROS_ERR ? (errno ? errno : EIO) : 0;
  return lastError == 0;
}

std::optional<AIVisionDetection> AiVisionSensor::convertObject(const pros::AIVision::Object& object) const {
  AIVisionDetection detection;
  detection.id = object.id;
  switch (static_cast<pros::AivisionDetectType>(object.type)) {
    
    case pros::AivisionDetectType::object: {
      detection.type = AIVisionDetectionType::Object;
      const auto& box = object.object.element;
      detection.x = box.xoffset;
      detection.y = box.yoffset;
      detection.width = box.width;
      detection.height = box.height;
      detection.confidence = box.score;
      break;
    }
    case pros::AivisionDetectType::color:
    case pros::AivisionDetectType::code: {
      detection.type = object.type == static_cast<uint8_t>(pros::AivisionDetectType::color)
          ? AIVisionDetectionType::Color : AIVisionDetectionType::Code;
      const auto& box = object.object.color;
      detection.x = box.xoffset;
      detection.y = box.yoffset;
      detection.width = box.width;
      detection.height = box.height;
      break;
    }
    case pros::AivisionDetectType::tag: {
      detection.type = AIVisionDetectionType::Tag;
      const auto& tag = object.object.tag;
      detection.x = std::min({tag.x0, tag.x1, tag.x2, tag.x3});
      detection.y = std::min({tag.y0, tag.y1, tag.y2, tag.y3});
      detection.width = std::max({tag.x0, tag.x1, tag.x2, tag.x3}) - detection.x;
      detection.height = std::max({tag.y0, tag.y1, tag.y2, tag.y3}) - detection.y;
      break;
    }
    default: return std::nullopt;
  }
  detection.centerX = detection.x + detection.width / 2.0;
  detection.centerY = detection.y + detection.height / 2.0;
  return detection;
}

int AiVisionSensor::getDetectionCount() {
  errno = 0;
  const auto count = sensor.get_object_count();
  if (errno || count == PROS_ERR || count < 0 || count > AIVISION_MAX_OBJECT_COUNT) {
    lastError = errno ? errno : EIO;
    return -1;
  }
  lastError = 0;
  return count;
}

AIVisionReadResult AiVisionSensor::readDetections() {
  const int count = getDetectionCount();
  if (count < 0) return {{}, lastError};
  AIVisionReadResult result;
  auto& detections = result.detections;
  detections.reserve(count);
  for (int index = 0; index < count; ++index) {
    errno = 0;
    const auto raw = sensor.get_object(index);
    const int readError = errno;
    const auto detection = convertObject(raw);
    if (readError || !detection) {
      lastError = readError ? readError : EIO;
      return {{}, lastError};
    }
    detections.push_back(*detection);
  }
  return result;
}

AIVisionLookupResult AiVisionSensor::readDetection(
    int id, std::optional<AIVisionDetectionType> type) {
  const auto read = readDetections();
  AIVisionLookupResult result{std::nullopt, read.error};
  if (!read.succeeded()) return result;
  std::int64_t largestArea = -1;
  for (const auto& detection : read.detections) {
    if (detection.id != id || (type && detection.type != *type)) continue;
    const auto area = static_cast<std::int64_t>(detection.width) * detection.height;
    if (!result.detection || area > largestArea) {
      result.detection = detection;
      largestArea = area;
    }
  }
  return result;
}

std::vector<AIVisionDetection> AiVisionSensor::getDetections() {
  return readDetections().detections;
}

std::optional<AIVisionDetection> AiVisionSensor::getDetection(
    int id, std::optional<AIVisionDetectionType> type) {
  return readDetection(id, type).detection;
}

}  // namespace aon
