#pragma once

#include "../math/pose.hpp"
#include <utility>
#include <math.h>
#include <float.h>
#include <vector>
#include <algorithm>
#include "./s-curve-profile.hpp"

namespace aon {

/// Experimental forward differential path follower. Inches, clockwise degrees,
/// motor RPM. Final-heading alignment belongs to the movement layer.
/// The path is a contiguous sequence of poses;
//  the controller does not search for the nearest point. The lookahead distance is independent of the path sampling;
//  the controller will interpolate between points to find the lookahead target. The controller is forward-only;
//  it will not reverse to reach a target behind the robot. The controller is not thread-safe;
//  the caller must ensure that only one thread calls `setPath` or `update` at a time.
class PurePursuit {
 public:
  enum class Status {
    IDLE, RUNNING, SUCCESS, TIMEOUT, CANCELLED, INVALID_PATH,
    UNSUPPORTED_CONFIGURATION, CONTROL_FAILURE
  };

  struct Diagnostics {
    Pose reference, lookahead;
    double progress = 0;       // inches, never decreases within a request
    double length = 0;
    double crossTrack = 0;     // inches from contiguous-segment projection
    double curvature = 0;     // clockwise 1/inch
    double targetRPM = 0;      // nonnegative center speed
    double measuredSpeed = 0;  // pose-derived inches/s
    double angularVelocity = 0; // commanded clockwise radians/s
  };

 private:
  // Motion Profiles
  MotionProfile linearProfile;
  MotionProfile angularProfile;

  double lookaheadDistance;  // inches along the path, independent of sampling
  std::vector<Pose> points;
  std::vector<double> distances, speedCaps;
  size_t segment = 0;
  Status pathStatus = Status::IDLE;
  Diagnostics diagnostics;
  Pose previousPose;
  bool hasPreviousPose = false;
  double progressAllowance = 0; // absolute inches; one acquisition + twice pose travel
  bool recoveringProgress = false;
  Pose recoveryReference;
  double recoveryTolerance = 0;
  double settledSeconds = 0;
  double maxLateralAcceleration = 30; // in/s^2; experimental, tune on robot

  static bool finitePose(const Pose& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
  }

  std::pair<double, double> fail(Status reason) {
    pathStatus = reason;
    diagnostics.targetRPM = diagnostics.angularVelocity = 0;
    return {0, 0};
  }

  double linearDeadband;  // Tune
  double angularDeadband;  // Tune

  bool validControlInput(const Pose& target, const Pose& current, double dt) const {
    return finitePose(target) && finitePose(current) && std::isfinite(dt) &&
        dt > 0 && dt <= 0.25 && linearProfile.isValid() && angularProfile.isValid() &&
        std::isfinite(linearDeadband) && linearDeadband > 0 &&
        std::isfinite(angularDeadband) && angularDeadband > 0 &&
        std::isfinite(DRIVE_WIDTH) && DRIVE_WIDTH > 0 &&
        std::isfinite(math::linearSpeed(1)) && math::linearSpeed(1) > 0;
  }

  std::pair<double, double> boundedLegacy(double left, double right) {
    if (!std::isfinite(left) || !std::isfinite(right))
      return fail(Status::CONTROL_FAILURE);
    const double limit = std::min(double(MAX_RPM),linearProfile.getMaxVelocity());
    const double scale = std::max({1.0,std::abs(left)/limit,std::abs(right)/limit});
    return {left/scale,right/scale};
  }

 public:
  PurePursuit(MotionProfile linearProfile, MotionProfile angularProfile,
              int lookaheadOffset, double linearDeadband, double angularDeadband)
      : linearProfile(linearProfile), angularProfile(angularProfile) {
    this->linearProfile.setFinalVelocity(0);
    this->angularProfile.setFinalVelocity(0);
    this->lookaheadDistance = lookaheadOffset;
    this->linearDeadband = linearDeadband;
    this->angularDeadband = angularDeadband;
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose`
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> go(Pose target, Pose current, double dt = 0.02) {
    // Basic Pure Pursuit-style controller (simplified for single target)
    if (!validControlInput(target,current,dt)) return fail(Status::CONTROL_FAILURE);

    // Extract positions
    double dx = target.x - current.x;
    double dy = target.y - current.y;

    // linearError to target
    double linearError = std::hypot(dx, dy);
    if (!std::isfinite(linearError)) return fail(Status::CONTROL_FAILURE);

    // Desired heading
    double targetAngle = std::atan2(dy, dx) * 180 / M_PI;

    // Current heading in degrees, reduced before computing the error.
    double currentHeading = std::remainder(current.theta,360.0);

    // Heading angularError (normalize to [-180, 180])
    double angularError = std::remainder(targetAngle - currentHeading,360.0);

    double linearSign = (linearError == 0) ? 0 : (linearError / std::abs(linearError));
    double angularSign = (angularError == 0) ? 0 : (angularError / std::abs(angularError));
    
    // Linear and angular velocities

    // TODO: check if using these three lines instead of the next one maintains accuracy while reducing angular oscillations
    // double angleFactor = std::cos(angularError * M_PI / 180.0);
    // angleFactor = std::clamp(angleFactor, 0.0, 1.0);
    // double linearVel = linearProfile.update(abs(linearError), dt) * * linearSign * angleFactor;
    double linearVel = linearProfile.update(std::abs(linearError), dt) * linearSign;
    
    const double circumference = DRIVE_WIDTH * M_PI;
    double angularArc = circumference * (std::abs(angularError) / 360.0);
    
    double angularVel = angularProfile.update(angularArc, dt) * angularSign;

    // TODO: try using these to see if there is any improvement but I (Kevin G) dont expect it
    // double curvature = (2 * sin(angularErrorRad)) / lookaheadDistance;
    // double angularVel = curvature * linearVel;

    // Convert to tank drive velocities
    // left = v + w, right = v - w
    double left = linearVel + angularVel;
    double right = linearVel - angularVel;

    // Deadband to avoid infinite loop
    if (std::abs(linearError) <= linearDeadband)
      return {0, 0};

    return boundedLegacy(left,right);
  }

  /// @brief Calculates the action from the `current` `Pose` to the `target` `Pose` to align heading
  /// @param target The `Pose` we want the robot to get to
  /// @param current  The `Pose` the robot is currently at
  /// @return A pair of \b RPM commands for the left and right sides of the drivetrain
  std::pair<double, double> turn(Pose target, Pose current, double dt = 0.02) {
    if (!validControlInput(target,current,dt)) return fail(Status::CONTROL_FAILURE);
    // TODO: determine if this is necessary
    // Current heading
    double currentHeading = std::remainder(current.theta,360.0);

    // Desired final heading
    double targetHeading = std::remainder(target.theta,360.0);

    // Reduce each finite heading before subtraction, avoiding overflow and loops.
    // The resulting angular error is in degrees, in [-180, 180].
    double angularError = std::remainder(targetHeading - currentHeading,360.0);

    double sign = (angularError == 0) ? 0 : (angularError / abs(angularError));

    // Pure turning: no linear velocity
    double angularVel = angularProfile.update(std::abs(angularError), dt) * sign;

    // Deadband (symmetric for turning)
    if (std::abs(angularError) <= angularDeadband) return {0, 0};

    return boundedLegacy(angularVel,-angularVel);
  }

  Status status() const { return pathStatus; }
  const Diagnostics& getDiagnostics() const { return diagnostics; }
  double finalHeading() const { return points.empty() ? 0 : points.back().theta; }

  /// @brief Copy and validate once before movement. Caller must explicitly start
  /// each new request; follow() never switches an active request's path.
  /// @param caps Optional waypoint center-speed ceilings in equivalent motor RPM, independent of
  /// export format. Positive except optional terminal zero. Reverse is unsupported.
  Status setPath(const std::vector<Pose>& path,
                 const std::vector<double>& caps = {}, bool reversed = false) {
    points.clear(); distances.clear(); speedCaps.clear();
    diagnostics = Diagnostics{};
    segment = 0; settledSeconds = 0; hasPreviousPose = false;
    progressAllowance = lookaheadDistance;
    recoveringProgress = false;
    pathStatus = Status::RUNNING;
    linearProfile.setVelocity(0); linearProfile.setAccel(0);
    angularProfile.setVelocity(0); angularProfile.setAccel(0);
    if (reversed) { fail(Status::UNSUPPORTED_CONFIGURATION); return pathStatus; }
    if (!linearProfile.isValid() || !angularProfile.isValid() ||
        !std::isfinite(lookaheadDistance) || lookaheadDistance <= 0 ||
        !std::isfinite(linearDeadband) || linearDeadband <= 0 ||
        !std::isfinite(angularDeadband) || angularDeadband <= 0 ||
        !std::isfinite(math::linearSpeed(1)) || math::linearSpeed(1) <= 0 ||
        !std::isfinite(DRIVE_WIDTH) || DRIVE_WIDTH <= 0) {
      fail(Status::CONTROL_FAILURE); return pathStatus;
    }
    // Bound setup memory and arithmetic on V5; field coordinates are far smaller.
    if (path.empty() || path.size() > 4096 ||
        (!caps.empty() && caps.size() != path.size())) {
      fail(Status::INVALID_PATH); return pathStatus;
    }
    for (size_t i = 0; i < path.size(); ++i) {
      const Pose& p = path[i];
      double cap = caps.empty() ? linearProfile.getMaxVelocity() : caps[i];
      if (!finitePose(p) || std::abs(p.x) > 1e6 || std::abs(p.y) > 1e6 ||
          !std::isfinite(cap) || cap < 0 || (cap == 0 && i + 1 < path.size())) {
        fail(Status::INVALID_PATH); return pathStatus;
      }
      cap = std::min({cap, linearProfile.getMaxVelocity(), double(MAX_RPM)});
      if (!points.empty() && points.back().distanceTo(p) <= 1e-9) {
        points.back() = p; // preserve final heading and stricter duplicate cap
        speedCaps.back() = std::min(speedCaps.back(), cap);
      } else {
        distances.push_back(points.empty() ? 0 :
            distances.back() + points.back().distanceTo(p));
        points.push_back(p); speedCaps.push_back(cap);
      }
    }
    diagnostics.length = distances.back();
    // Backward braking envelope respects future exported restrictions. Conversion
    // is RPM -> in/s before applying v^2 = u^2 + 2*a*s.
    speedCaps.back() = 0;
    const double unit = math::linearSpeed(1);
    const double decel = math::linearSpeed(linearProfile.getMaxDeceleration() * 0.5);
    for (size_t i = points.size() - 1; i > 0; --i) {
      speedCaps[i-1] = std::min(speedCaps[i-1],
          std::sqrt(std::pow(speedCaps[i] * unit, 2) +
                    2 * decel * (distances[i] - distances[i-1])) / unit);
    }
    return pathStatus;
  }

  /// @brief Advance the prepared path without allocation or I/O. Settlement
  /// requires 150ms near the endpoint and end of progress, below 1in/s and
  /// 5deg/s pose motion. Success covers translation, never final alignment.
  std::pair<double, double> update(Pose current, double dt = 0.02) {
    if (pathStatus != Status::RUNNING) return {0, 0};
    if (!finitePose(current) || !std::isfinite(dt) || dt <= 0 || dt > 0.25)
      return fail(Status::CONTROL_FAILURE);

    const double moved = hasPreviousPose ? current.distanceTo(previousPose) : 0;
    const double angularSpeed = hasPreviousPose ?
        std::abs(std::remainder(std::remainder(current.theta, 360.0) -
            std::remainder(previousPose.theta, 360.0), 360.0)) / dt : 0;
    diagnostics.measuredSpeed = moved / dt;
    const bool canSettle = hasPreviousPose;
    const Pose lastPose = previousPose;
    previousPose = current; hasPreviousPose = true;
    if (!std::isfinite(moved) || !std::isfinite(diagnostics.measuredSpeed) ||
        !std::isfinite(angularSpeed)) return fail(Status::CONTROL_FAILURE);

    // Follow only contiguous segments, never a global nearest-point search.
    // The acquisition allowance is issued once. Corner cuts may consume up to
    // two path inches per measured inch, but no corner can replenish this bound.
    progressAllowance = std::min(diagnostics.length,progressAllowance + 2*moved);
    const double progressLimit = progressAllowance;
    // Non-collinear branches require directed corner-bisector crossings, in
    // increasing order along this pose displacement. A pose already past an
    // unvisited gate cannot repeatedly authorize later branches. Exact reversals
    // use the incoming endpoint plane because their bisector has zero length.
    double lastCrossing = -1;
    bool progressLimited = false;
    bool missedGate = false;
    Pose gateRecoveryReference;
    double gateRecoveryTolerance = 0;
    while (segment + 1 < points.size()) {
      const Pose& a = points[segment]; const Pose& b = points[segment+1];
      const double length = distances[segment+1] - distances[segment];
      double t = ((current.x-a.x)*(b.x-a.x)+(current.y-a.y)*(b.y-a.y)) /
                 (length*length);
      double projection = distances[segment] + std::clamp(t, 0.0, 1.0)*length;
      progressLimited = projection > progressLimit+1e-9;
      diagnostics.progress = std::max(diagnostics.progress,
          std::min(projection, progressLimit));
      if (segment + 2 == points.size()) break;
      const Pose& next = points[segment+2];
      const double nextLength = distances[segment+2]-distances[segment+1];
      const double ux = (b.x-a.x)/length, uy = (b.y-a.y)/length;
      const double vx = (next.x-b.x)/nextLength, vy = (next.y-b.y)/nextLength;
      const double dot = ux*vx+uy*vy;
      if (dot > 1-1e-6) {
        if (diagnostics.progress < distances[segment+1]-1e-9) break;
      } else {
        double nx = ux+vx, ny = uy+vy;
        if (nx*nx+ny*ny <= 1e-12) { nx=ux; ny=uy; }
        const double before = (lastPose.x-b.x)*nx+(lastPose.y-b.y)*ny;
        const double after = (current.x-b.x)*nx+(current.y-b.y)*ny;
        const bool directed = canSettle && moved > 1e-9 && before <= 1e-9 &&
            after >= -1e-9 && after-before > 1e-9;
        const double crossing = directed ? std::clamp(-before/(after-before),0.0,1.0) : 0;
        const bool ordered = directed && crossing > lastCrossing+1e-9;
        if (after > 1e-9 && !ordered) {
          missedGate = true;
          // Retreat along the gate normal, not merely along the incoming edge:
          // near reversals that edge can run almost parallel to the gate.
          const double retreat = std::min(lookaheadDistance,length)*0.5;
          const double norm = std::hypot(nx,ny);
          gateRecoveryReference = Pose(b.x-nx/norm*retreat,b.y-ny/norm*retreat);
          // This arrival ball lies strictly before the gate (radius <= retreat/2).
          gateRecoveryTolerance = std::min(linearDeadband,retreat*0.5);
        }
        if (distances[segment+1] > progressLimit && after >= -1e-9)
          progressLimited = true;
        if (!canSettle || moved <= 1e-9 ||
            distances[segment+1] > progressLimit ||
            distances[segment+1]-diagnostics.progress > lookaheadDistance) break;
        if (!ordered) break;
        lastCrossing = crossing;
        diagnostics.progress = distances[segment+1];
      }
      ++segment;
    }
    diagnostics.reference = points.front();
    if (points.size() > 1) {
      double t = (diagnostics.progress-distances[segment]) /
                 (distances[segment+1]-distances[segment]);
      diagnostics.reference = points[segment] + (points[segment+1]-points[segment])*t;
    }
    diagnostics.crossTrack = current.distanceTo(diagnostics.reference);
    const double remaining = diagnostics.length - diagnostics.progress;
    const double endError = current.distanceTo(points.back());
    if (!std::isfinite(endError) || !std::isfinite(diagnostics.crossTrack) ||
        !std::isfinite(diagnostics.measuredSpeed) || !std::isfinite(angularSpeed))
      return fail(Status::CONTROL_FAILURE);
    if (remaining <= linearDeadband && endError <= linearDeadband) {
      diagnostics.targetRPM = diagnostics.angularVelocity = 0;
      settledSeconds = canSettle && diagnostics.measuredSpeed <= 1 && angularSpeed <= 5 ?
          settledSeconds + dt : 0;
      if (settledSeconds >= 0.15) pathStatus = Status::SUCCESS;
      return {0, 0};
    }
    settledSeconds = 0;
    double targetDistance = std::min(diagnostics.length,
        diagnostics.progress + lookaheadDistance);
    // Do not aim through an untraversed fold back onto the incoming branch.
    // The bound depends on segment direction, not waypoint count, so adding
    // collinear samples does not change it. Ordinary corner cutting is retained.
    if (segment+1 < points.size()) {
      const double length = distances[segment+1]-distances[segment];
      const double ux = (points[segment+1].x-points[segment].x)/length;
      const double uy = (points[segment+1].y-points[segment].y)/length;
      for (size_t i=segment+1; i+1 < points.size() && distances[i]<targetDistance; ++i) {
        const double nextLength = distances[i+1]-distances[i];
        const double vx = (points[i+1].x-points[i].x)/nextLength;
        const double vy = (points[i+1].y-points[i].y)/nextLength;
        if (ux*vx+uy*vy < -1e-6) { targetDistance=distances[i]; break; }
      }
    }
    size_t ahead = segment;
    while (ahead + 1 < points.size() && distances[ahead+1] < targetDistance) ++ahead;
    diagnostics.lookahead = points.back();
    if (ahead + 1 < points.size()) {
      const double t = (targetDistance-distances[ahead]) /
          (distances[ahead+1]-distances[ahead]);
      diagnostics.lookahead = points[ahead] + (points[ahead+1]-points[ahead])*t;
    }
    // Recover to a fixed earned reference before looking forward again. Latch
    // the target until arrival: alternating between acquisition and lookahead
    // as a projection touches its allowance can otherwise strand the controller
    // rotating between two targets without earning any further displacement.
    if ((progressLimited || missedGate) && !recoveringProgress) {
      recoveryReference = missedGate ? gateRecoveryReference : diagnostics.reference;
      recoveryTolerance = missedGate ? gateRecoveryTolerance :
          std::min(linearDeadband,lookaheadDistance*0.25);
      recoveringProgress = true;
    }
    if (recoveringProgress) {
      if (current.distanceTo(recoveryReference) <= recoveryTolerance) recoveringProgress = false;
      else diagnostics.lookahead = recoveryReference;
    }
    // A folded or closed path can place its arc-length target exactly at the
    // robot. Recover on the active branch instead of faulting or skipping it.
    if (current.distanceTo(diagnostics.lookahead) <= 1e-6 &&
        segment+1 < points.size()) {
      diagnostics.lookahead = points[segment+1];
      if (current.distanceTo(diagnostics.lookahead) <= 1e-6)
        diagnostics.lookahead = segment+2 < points.size() ?
            points[segment+2] : diagnostics.reference;
    }
    const double dx = diagnostics.lookahead.x-current.x;
    const double dy = diagnostics.lookahead.y-current.y;
    const double heading = std::remainder(current.theta,360.0)*M_PI/180;
    const double lateral = -std::sin(heading)*dx + std::cos(heading)*dy;
    const double forward = std::cos(heading)*dx + std::sin(heading)*dy;
    const double chordSquared = dx*dx+dy*dy;
    if (chordSquared <= 1e-12) return fail(Status::CONTROL_FAILURE);
    diagnostics.curvature = 2*lateral/chordSquared;
    double rpm = linearProfile.update(std::max(remaining,endError), dt);
    const double unit = math::linearSpeed(1);
    double cap = linearProfile.getMaxVelocity();
    if (segment + 1 < points.size()) {
      const double decel = math::linearSpeed(linearProfile.getMaxDeceleration()*0.5);
      // Projected end progress does not imply arrival: retain braking distance
      // to the actual endpoint while recovering a lateral offset or overshoot.
      const double brakingDistance = segment+2 == points.size() ?
          std::max(distances[segment+1]-diagnostics.progress,endError) :
          distances[segment+1]-diagnostics.progress;
      cap = std::min(speedCaps[segment], std::sqrt(
          std::pow(speedCaps[segment+1]*unit,2) +
          2*decel*brakingDistance)/unit);
    }
    const double curveCap = std::sqrt(maxLateralAcceleration /
        std::max(std::abs(diagnostics.curvature),1e-9))/unit;
    const double error = std::atan2(lateral,forward);
    rpm = std::min({rpm, cap, curveCap, double(MAX_RPM)}) *
          std::max(0.0,std::cos(error));
    double turnRPM = rpm*diagnostics.curvature*DRIVE_WIDTH/2;
    // Forward-only tracking needs a stationary acquisition turn when the target
    // lies behind; curvature*v alone would lock the robot at zero center speed.
    if (std::abs(error) >= M_PI/2) {
      rpm = 0;
      turnRPM = std::copysign(angularProfile.update(
          std::abs(error)*DRIVE_WIDTH/2,dt), error);
    } else { angularProfile.setVelocity(0); angularProfile.setAccel(0); }
    double left = rpm+turnRPM, right = rpm-turnRPM;
    const double wheelLimit = std::min(double(MAX_RPM),linearProfile.getMaxVelocity());
    const double scale = std::max({1.0,std::abs(left)/wheelLimit,std::abs(right)/wheelLimit,
        std::abs(turnRPM)/angularProfile.getMaxVelocity()});
    left /= scale; right /= scale;
    if (!std::isfinite(left) || !std::isfinite(right) || rpm < 0)
      return fail(Status::CONTROL_FAILURE);
    diagnostics.targetRPM = rpm/scale;
    diagnostics.angularVelocity = (left-right)*unit/DRIVE_WIDTH;
    // Feed applied center speed back to avoid profile windup behind speed caps.
    linearProfile.setVelocity(diagnostics.targetRPM);
    return {left,right};
  }

  /// @brief Compatibility entry point. Path is prepared on the first call only;
  /// use setPath explicitly before reusing a controller for another request.
  std::pair<double, double> follow(const std::vector<Pose>& path, Pose current,
                                  double dt = 0.02) {
    if (pathStatus == Status::IDLE) setPath(path);
    return update(current,dt);
  }
};

}  // namespace aon
