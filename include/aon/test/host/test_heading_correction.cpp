#include "controls/heading-correction.hpp"
#include <cstdio>
#include <functional>
using namespace aon;

static int fails = 0, checks = 0;
#define CHECK(c, ...) do { ++checks; if (!(c)) { ++fails; printf("  FAIL: "); printf(__VA_ARGS__); printf("\n"); } } while (0)

// Crude first-order turning plant. NOT a model of your robot; it only exercises the logic.
struct Sim {
  double h, w = 0, gain = 1.5, tau = 0.08;   // true heading (continuous), rate deg/s
  std::uint32_t t = 0;
  double cmd = 0, maxAbsCmd = 0, peakOvershoot = 0, target = 0, startSign = 0;
  int stopCalls = 0; bool stuck = false;
  std::function<bool(std::uint32_t)> cancelWhen = [](std::uint32_t) { return false; };
  std::function<bool(std::uint32_t)> sensorBad = [](std::uint32_t) { return false; };
  explicit Sim(double h0) : h(h0) {}
  std::uint32_t nowMs() { return t; }
  double headingDeg() {
    if (sensorBad(t)) return NAN;
    double r = std::fmod(h, 360.0); return r < 0 ? r + 360.0 : r;   // odometry-like 0..360
  }
  void rotate(double rpm) { cmd = rpm; maxAbsCmd = std::max(maxAbsCmd, std::fabs(rpm)); }
  void stop() { cmd = 0; ++stopCalls; }
  bool cancelled() { return cancelWhen(t); }
  void delayMs(std::uint32_t ms) {
    for (std::uint32_t i = 0; i < ms; i++) {
      double wt = stuck ? 0 : gain * cmd;
      w += (wt - w) * (0.001 / tau);
      h += w * 0.001; ++t;
      double e = shortestAngleError(target, h);
      if (startSign * e < 0) peakOvershoot = std::max(peakOvershoot, std::fabs(e));
    }
  }
  double trueError() const { return shortestAngleError(target, h); }
};

static HeadingCorrectionConfig cfg() {
  HeadingCorrectionConfig c;
  c.kP = 0.02; c.kD = 0.0005; c.maxTurnRpm = 100;
  c.toleranceDeg = 2; c.settleMs = 150; c.timeoutMs = 3000;
  return c;
}
static HeadingResult run(Sim& s, double target, HeadingCorrectionConfig c = cfg(),
                         MovementStatus p = MovementStatus::SUCCESS) {
  s.target = target; s.startSign = shortestAngleError(target, s.h) >= 0 ? 1 : -1;
  return correctHeading(target, c, s, p);
}

int main() {
  printf("-- shortestAngleError --\n");
  struct { double t, c, e; } cases[] = {{1,359,2},{359,1,-2},{0,180,-180+360},{180,0,180},{180,360,180},
      {90,725,85},{-30,330,0},{10,350,20},{350,10,-20},{0,90,-90},{1e6,0,-80}};
  for (auto& k : cases) {
    double g = shortestAngleError(k.t, k.c);
    CHECK(std::fabs(g - k.e) < 1e-9 || (std::fabs(std::fabs(g) - 180) < 1e-9 && std::fabs(k.e) == 180), "err(%g,%g)=%g want %g", k.t, k.c, g, k.e);
  }
  { double g = shortestAngleError(0, 180); CHECK(g == 180.0, "180 apart must be +180, got %g", g); }

  printf("-- SettleTracker --\n");
  { SettleTracker s(2, 100);
    CHECK(!s.update(1, 0), "needs settle time"); CHECK(!s.update(1, 99), "99ms not enough");
    CHECK(s.update(1, 100), "100ms ok");
    s.reset(); s.update(1, 0); CHECK(!s.update(3, 50), "leaving tol resets"); CHECK(!s.update(1, 60), "restarts");
    CHECK(!s.update(1, 150), "only 90ms since re-entry"); CHECK(s.update(1, 160), "100ms since re-entry");
    SettleTracker z(2, 0); CHECK(z.update(0, 5), "settle 0 succeeds at once");
    SettleTracker w(2, 100); w.update(1, 0xFFFFFFF0u); CHECK(w.update(1, 0x60), "uint32 wrap handled"); }

  printf("-- validation --\n");
  { Sim s(0); HeadingCorrectionConfig bad;   // zero-initialised
    CHECK(run(s, 10, bad) == HeadingResult::INVALID_PARAMS, "default cfg rejected");
    auto c = cfg(); c.kD = -1; CHECK(run(s, 10, c) == HeadingResult::INVALID_PARAMS, "neg kD");
    c = cfg(); c.toleranceDeg = 0; CHECK(run(s, 10, c) == HeadingResult::INVALID_PARAMS, "tol 0");
    c = cfg(); c.timeoutMs = 0; CHECK(run(s, 10, c) == HeadingResult::INVALID_PARAMS, "timeout 0");
    c = cfg(); c.maxTurnRpm = NAN; CHECK(run(s, 10, c) == HeadingResult::INVALID_PARAMS, "NaN rpm");
    CHECK(run(s, NAN) == HeadingResult::INVALID_PARAMS, "NaN target");
    CHECK(run(s, INFINITY) == HeadingResult::INVALID_PARAMS, "inf target");
    CHECK(s.maxAbsCmd == 0, "no motion on invalid input"); }

  printf("-- 1 minimal error (no motion) --\n");
  { Sim s(0.5); auto r = run(s, 0); CHECK(r == HeadingResult::SUCCESS, "got %s", toString(r));
    CHECK(s.maxAbsCmd == 0, "robot was commanded %.1f rpm despite being in tolerance", s.maxAbsCmd);
    CHECK(s.t >= 150 && s.t <= 170, "should take about settle time, took %u ms", s.t); }

  printf("-- 3 undershoot / normal correction --\n");
  { Sim s(60); auto r = run(s, 90); CHECK(r == HeadingResult::SUCCESS, "got %s", toString(r));
    CHECK(std::fabs(s.trueError()) <= 2.0, "final err %.2f", s.trueError()); printf("  30deg: %u ms, overshoot %.2f deg\n", s.t, s.peakOvershoot); }

  printf("-- 2 overshoot (arrives with momentum) --\n");
  { Sim s(80); s.w = 120; auto r = run(s, 90); CHECK(r == HeadingResult::SUCCESS, "got %s", toString(r));
    CHECK(std::fabs(s.trueError()) <= 2.0, "final err %.2f", s.trueError()); printf("  coasting in: overshoot %.2f deg, %u ms\n", s.peakOvershoot, s.t); }

  printf("-- 4 wrap boundary --\n");
  { Sim s(350); auto r = run(s, 10); CHECK(r == HeadingResult::SUCCESS, "got %s", toString(r));
    CHECK(s.h > 350, "should go clockwise through 360, ended at true heading %.1f", s.h);
    Sim u(10); r = run(u, 350); CHECK(r == HeadingResult::SUCCESS, "got %s", toString(r));
    CHECK(u.h < 10, "should go counter-clockwise, ended at %.1f", u.h); }

  printf("-- 5 large error --\n");
  { Sim s(0); auto r = run(s, 170); CHECK(r == HeadingResult::SUCCESS, "got %s", toString(r));
    CHECK(s.maxAbsCmd <= 100.0 + 1e-9, "cap exceeded: %.1f", s.maxAbsCmd); printf("  170deg: %u ms\n", s.t); }

  printf("-- 6 timeout (plant ignores commands) --\n");
  { Sim s(0); s.stuck = true; auto c = cfg(); c.timeoutMs = 500; auto r = run(s, 40, c);
    CHECK(r == HeadingResult::TIMEOUT, "got %s", toString(r));
    CHECK(s.t >= 500 && s.t <= 520, "timeout at %u ms", s.t); CHECK(s.cmd == 0 && s.stopCalls == 1, "motors not stopped"); }

  printf("-- 7 cancellation mid-correction --\n");
  { Sim s(0); s.cancelWhen = [](std::uint32_t t) { return t >= 200; }; auto r = run(s, 120);
    CHECK(r == HeadingResult::CANCELLED, "got %s", toString(r));
    CHECK(s.t <= 220, "stopped late at %u ms", s.t); CHECK(s.cmd == 0 && s.stopCalls == 1, "motors not stopped");
    Sim e(0); e.cancelWhen = [](std::uint32_t) { return true; }; r = run(e, 90);
    CHECK(r == HeadingResult::CANCELLED && e.maxAbsCmd == 0, "cancelled on entry must not move"); }

  printf("-- sensor faults --\n");
  { Sim s(0); s.sensorBad = [](std::uint32_t) { return true; }; auto r = run(s, 30);
    CHECK(r == HeadingResult::INVALID_SENSOR && s.maxAbsCmd == 0, "bad from start: %s", toString(r));
    Sim g(0); g.sensorBad = [](std::uint32_t t) { return t == 100; }; r = run(g, 90);
    CHECK(r == HeadingResult::SUCCESS, "1-sample glitch should recover, got %s", toString(r));
    Sim d(0); d.sensorBad = [](std::uint32_t t) { return t >= 100 && t < 160; }; r = run(d, 90);
    CHECK(r == HeadingResult::INVALID_SENSOR, "3 bad samples should fail, got %s", toString(r));
    CHECK(d.cmd == 0 && d.stopCalls == 1, "stopped on sensor failure"); }

  printf("-- prior movement failed -> skipped --\n");
  { Sim s(0); auto r = run(s, 90, cfg(), MovementStatus::TIMEOUT);
    CHECK(r == HeadingResult::SKIPPED && s.maxAbsCmd == 0, "got %s", toString(r));
    r = run(s, 90, cfg(), MovementStatus::CANCELLED); CHECK(r == HeadingResult::SKIPPED, "cancelled -> skipped");
    r = run(s, 90, cfg(), MovementStatus::STALLED); CHECK(r == HeadingResult::SKIPPED && s.maxAbsCmd == 0, "stalled -> skipped"); }

  printf("-- 8 sequential corrections --\n");
  { Sim s(0); double ts[] = {90, 180, 0}; int ok = 0;
    for (double t : ts) { if (run(s, t) == HeadingResult::SUCCESS && std::fabs(s.trueError()) <= 2.0) ok++; }
    CHECK(ok == 3, "only %d/3 succeeded", ok); CHECK(s.stopCalls == 3, "one stop per correction, got %d", s.stopCalls); }

  printf("\n%d checks, %d failed\n", checks, fails);
  return fails != 0;
}
