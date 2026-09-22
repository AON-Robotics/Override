#include "../../include/aon/sensing/pi-link.hpp"

#include <cctype>
#include <cmath>
#include <cstdio>

namespace aon {

namespace {
constexpr std::size_t kMaxBufferedChars = 64;  // guards against a malformed stream with no newline
constexpr std::uint32_t kPacketTimeoutMs = 300;
}  // namespace

void PiLink::run() {
  while (true) {
    const int c = std::fgetc(stdin);  // blocks this task until the Pi sends a byte
    if (c == EOF) {
      std::clearerr(stdin);
      pros::delay(10);
      continue;
    }

    if (c == '\n') {
      Reading reading;
      if (parseLine(buffer_, reading)) {
        mutex_.take(TIMEOUT_MAX);
        latest_ = reading;
        lastPacketMs_ = pros::millis();
        hasPacket_ = true;
        mutex_.give();
      }
      buffer_.clear();
    } else if (c != '\r') {
      buffer_ += static_cast<char>(c);
      if (buffer_.size() > kMaxBufferedChars) buffer_.clear();  // drop garbage, wait for next line
    }
  }
}

PiLink::Reading PiLink::latest() {
  mutex_.take(TIMEOUT_MAX);
  Reading copy;
  if (hasPacket_ && pros::millis() - lastPacketMs_ <= kPacketTimeoutMs) {
    copy = latest_;
  }
  mutex_.give();
  return copy;
}

bool PiLink::parseLine(const std::string& line, Reading& out) {
  if (line == "N,0") {
    out = Reading{};
    return true;
  }

  const std::size_t comma = line.find(',');
  if (comma != 1) return false;

  const char colorChar = static_cast<char>(std::toupper(static_cast<unsigned char>(line[0])));
  switch (colorChar) {
    case 'R': out.color = Color::Red; break;
    case 'B': out.color = Color::Blue; break;
    case 'G': out.color = Color::Green; break;
    default: return false;
  }

  const std::string distanceToken = line.substr(comma + 1);
  try {
    std::size_t charsParsed = 0;
    out.distanceInches = std::stod(distanceToken, &charsParsed);
    if (charsParsed != distanceToken.size() ||
        !std::isfinite(out.distanceInches) || out.distanceInches < 0.0) {
      return false;
    }
  } catch (...) {
    return false;
  }

  out.valid = true;
  return true;
}

}  // namespace aon
