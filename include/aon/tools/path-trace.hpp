#pragma once
#include "../controls/path.hpp"
#include <cstdio>
#include <memory>
#include <new>

namespace aon {

// Bounded, optional RAM recording. All filesystem work happens after stopping.
class PathTrace {
  struct Row {
    std::uint32_t ms;
    std::uint16_t leg;
    bool aligning;
    float values[13];
  };
  std::unique_ptr<Row[]> rows;
  std::size_t capacity, count = 0;
  std::uint32_t started;
  bool truncated = false;

public:
  explicit PathTrace(std::uint32_t startMs, std::size_t limit = 320)
      : rows(limit ? new(std::nothrow) Row[limit] : nullptr),
        capacity(rows ? limit : 0), started(startMs) {}

  void record(const FollowSample& value, std::uint32_t elapsed, std::uint16_t leg = 0) {
    if (count == capacity) { truncated = true; return; }
    rows[count++] = {elapsed,leg,value.aligning,
      {static_cast<float>(value.pose.x),static_cast<float>(value.pose.y),static_cast<float>(value.pose.theta),
       static_cast<float>(value.target.x),static_cast<float>(value.target.y),static_cast<float>(value.target.theta),
       static_cast<float>(value.progress),static_cast<float>(value.crossTrackError),static_cast<float>(value.endpointError),
       static_cast<float>(value.left),static_cast<float>(value.right),
       static_cast<float>(value.measuredLeft),static_cast<float>(value.measuredRight)}};
  }

  bool save(const char* base, const char* result, const Pose& actual, const Pose& endpoint,
            std::uint32_t elapsed, const FollowOptions* options = nullptr,
            std::uint32_t profile = 0, std::uint32_t revision = 0) const {
    char filename[128];
    const int length = std::snprintf(filename,sizeof(filename),"%s-runs.csv",base);
    if (length < 0 || static_cast<std::size_t>(length) >= sizeof(filename)) return false;
    auto* summary = std::fopen(filename,"a+");
    if (!summary) return false;
    std::fseek(summary,0,SEEK_END);
    const long id = std::ftell(summary);
    std::snprintf(filename,sizeof(filename),"%s.csv",base);
    auto* trace = std::fopen(filename,"a+");
    if (!trace) { std::fclose(summary); return false; }
    std::fseek(trace,0,SEEK_END);
    if (std::ftell(trace) == 0)
      std::fputs("run,leg,ms,x,y,heading,target_x,target_y,target_heading,progress,cross_track,endpoint_error,left_cmd,right_cmd,left_rpm,right_rpm,aligning\n",trace);
    for (std::size_t i=0; i<count; ++i) {
      const auto& row = rows[i];
      std::fprintf(trace,"%ld,%u,%lu",id,static_cast<unsigned>(row.leg),static_cast<unsigned long>(row.ms));
      for (float value : row.values) std::fprintf(trace,",%.3f",static_cast<double>(value));
      std::fprintf(trace,",%d\n",row.aligning);
    }
    if (id == 0) {
      std::fputs("run,start_ms,result,elapsed_ms,position_error,heading_error,samples,truncated",summary);
      if (options) std::fputs(
          ",profile,revision,wheel_diameter,gear_ratio,drive_width,tracking_diameter,base_accel,base_decel"
          ",maximumRpm,lookahead,lookaheadAtSpeed,positionTolerance,headingTolerance,lateralAcceleration"
          ",settleMs,settledRpm,timeoutMs,accelerationScale,decelerationScale,turnAccelerationScale,turnDecelerationScale",summary);
      std::fputc('\n',summary);
    }
    std::fprintf(summary,"%ld,%lu,%s,%lu,%.3f,%.3f,%u,%d",id,
                 static_cast<unsigned long>(started),result,static_cast<unsigned long>(elapsed),
                 actual.distanceTo(endpoint),std::abs(std::remainder(endpoint.theta-actual.theta,360)),
                 static_cast<unsigned>(count),truncated);
    if (options) {
      const auto& o = *options;
      std::fprintf(summary,",%lu,%lu",static_cast<unsigned long>(profile),static_cast<unsigned long>(revision));
      for (double value : {double(DRIVE_WHEEL_DIAMETER), double(MOTOR_TO_DRIVE_RATIO), double(DRIVE_WIDTH),
                           double(TRACKING_WHEEL_DIAMETER), double(MAX_ACCEL), double(MAX_DECEL),
                           o.maximumRpm, o.lookahead, o.lookaheadAtSpeed, o.positionTolerance, o.headingTolerance,
                           o.lateralAcceleration, double(o.settleMs), o.settledRpm, double(o.timeoutMs),
                           o.accelerationScale, o.decelerationScale, o.turnAccelerationScale, o.turnDecelerationScale})
        std::fprintf(summary,",%.12g",value);
    }
    std::fputc('\n',summary);
    const bool written = !std::ferror(trace) && !std::ferror(summary);
    const bool traceClosed = std::fclose(trace) == 0;
    const bool summaryClosed = std::fclose(summary) == 0;
    return written && traceClosed && summaryClosed;
  }
};
} // namespace aon
