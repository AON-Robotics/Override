#include "aon/competition/path-tuning.hpp"

void testTuning() {
  const char* filename = "tuning-test.csv";
  const auto write = [&](const char* suffix, double wheel = DRIVE_WHEEL_DIAMETER) {
    auto* file = std::fopen(filename,"w");
    assert(file);
    std::fprintf(file,"AON_PATH_TUNING_V1\n7,%.12g,%.12g,%.12g,%.12g,%.12g,%.12g,%s",
        wheel,double(MOTOR_TO_DRIVE_RATIO),double(DRIVE_WIDTH),double(TRACKING_WHEEL_DIAMETER),
        double(MAX_ACCEL),double(MAX_DECEL),suffix);
    assert(std::fclose(file) == 0);
  };
  aon::FollowOptions options;
  std::uint32_t id = 99;
  assert(aon::loadPathTuning(options,id,"missing-profile.csv") == aon::TuningLoad::Missing);
  assert(id == 99 && options.maximumRpm == 200);
  write("100,4,7,1,2,25,200,3,10000,0.5,0.8,0.7,0.6\n");
  assert(aon::loadPathTuning(options,id,filename) == aon::TuningLoad::Loaded);
  assert(id == 7 && options.maximumRpm == 100 && options.lookahead == 4 && options.lookaheadAtSpeed == 7);
  assert(options.accelerationScale == 0.5 && options.turnDecelerationScale == 0.6);
  assert(options.settleMs == 200 && options.timeoutMs == 10000);
  // Run the loaded profile through production following with an independent endpoint.
  pros::reset();
  SimDrive drive;
  pros::advance = [&](unsigned ms) { drive.advance(ms); };
  const aon::PathRoute route{{{0,0,0},{12,0,0},{24,0,0}},{100,60,0}};
  assert(drive.follow(route.view(),options) == aon::Drivetrain::FollowResult::Completed);
  assert(drive.getPose().distanceTo({24,0,0}) <= 1.1);
  pros::advance = nullptr;
  for (const char* invalid : {
      "nan,4,7,1,2,25,200,3,10000,1,1,1,1",
      "201,4,7,1,2,25,200,3,10000,1,1,1,1",
      "100,4,7,1,2,25,200.5,3,10000,1,1,1,1",
      "100,4,7,1,2,25,200,3,10000,0,1,1,1",
      "100,4,7,1,2,25,200,3,10000,1,1,1",
      "100,4,7,1,2,25,200,3,10000,1,1,1,1,extra"}) {
    write(invalid);
    assert(aon::loadPathTuning(options,id,filename) == aon::TuningLoad::Invalid);
    assert(id == 7 && options.maximumRpm == 100 && options.accelerationScale == 0.5);
  }
  write("100,4,7,1,2,25,200,3,10000,1,1,1,1",99);
  assert(aon::loadPathTuning(options,id,filename) == aon::TuningLoad::Invalid);
  std::remove(filename);
  aon::PathTrace trace(0);
  aon::FollowSample sample;
  trace.record(sample,300);
  assert(trace.save("tuning-trace","Completed",{0,0,30},{0,0,90},300,&options,7,123));
  std::ifstream file("tuning-trace-runs.csv");
  std::string header,row;
  std::getline(file,header); std::getline(file,row);
  assert(header.find("profile,revision,wheel_diameter") != std::string::npos);
  assert(row.find(",0.000,60.000,1,0,7,123,") != std::string::npos);
  assert(row.find(",100,4,7,1,2,25,200,3,10000,0.5,0.8,0.7,0.6") != std::string::npos);
}
