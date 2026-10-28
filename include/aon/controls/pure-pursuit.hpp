#pragma once

#include "../math/pose.hpp"
#include <utility>
#include <math.h>
#include <float.h>
#include <vector>
#include <algorithm>
#include <cstdint>
#include "./s-curve-profile.hpp"

namespace aon {

/**
 * @brief Experimental forward-center differential-drive polyline controller.
 * @details Positions, lookahead and track width use inches; headings use clockwise
 * degrees in the AON frame (+X forward at zero heading, +Y right). Commands are
 * equivalent motor RPM. Positive left-minus-right command turns clockwise.
 * Ordered, contiguous projection and distance interpolation determine progress
 * and lookahead; intermediate waypoint headings do not steer translation.
 * A tight turn may reverse the inner wheel, but commanded center motion never
 * reverses. Targets behind the robot require stationary heading acquisition.
 *
 * setPath() prepares owned storage; update() performs no allocation, formatting
 * or I/O. This class does not read sensors, publish motor commands, enforce a
 * wall-clock timeout, cancel requests or align the final heading. The movement
 * layer owns those operations and must inspect returned status/commands.
 * @pre Serialize every access to an instance, including diagnostics/status reads
 * and legacy go()/turn(); no internal synchronization protects mutable state.
 * @note Profiles shape center/acquisition requests. Geometry, speed ceilings and
 * stopping can override smoothness; final wheel jerk/acceleration is not globally
 * constrained. Legacy go()/turn() retain separate single-target behavior.
 * @todo Issue 18: measure tracking, stopping, loaded V5 timing and preparation
 * memory margin on calibrated hardware before competition use; software evidence
 * does not establish repeated-run physical accuracy.
 * @todo Issue 8: adapt exporter frames and speed metadata to setPath() outside
 * this controller. Issue 33: complete separately reviewed legacy movement
 * integration and physical tuning of the movement-owned final-heading stage.
 */
class PurePursuit {
 public:
  /// @brief Largest accepted preceding path-update interval, in seconds.
  /// @details The movement loop normally requests 10 ms. Longer holds exhibited
  /// sampled-target overshoot in software experiments; 50 ms is the supported
  /// experimental envelope, not a hard real-time or physical convergence guarantee.
  /// @todo Measure loaded V5 scheduling and reaction to blocked command holds.
  static constexpr double maxPathInterval = 0.05; // seconds; legacy go/turn unchanged
  /// @brief Shared controller/movement request lifecycle and failure vocabulary.
  /// @details This controller emits preparation/control/translation results;
  /// TIMEOUT and CANCELLED are movement-layer results, not update() transitions.
  enum class Status {
    IDLE, ///< No path has been prepared; follow() may prepare its first path.
    RUNNING, ///< Prepared request is active, including zero-command settlement.
    SUCCESS, ///< Translation settled; final-heading alignment is not implied.
    TIMEOUT, ///< Movement layer observed expiry of its cooperative stage budget.
    CANCELLED, ///< Movement layer observed cancellation of this request.
    INVALID_PATH, ///< Path coordinates, count or waypoint cap data are invalid.
    UNSUPPORTED_CONFIGURATION, ///< Unsupported drivetrain, reverse, or active actuator resolution.
    CONTROL_FAILURE ///< Invalid configuration/input or unusable control geometry.
  };

  /// @brief Latest geometry and floating command snapshot, not motor feedback.
  /// @details Read only under the instance's caller-owned serialization contract.
  /// motorCommands() can further reduce the published integer wheel pair.
  struct Diagnostics {
    /// @brief Earned-progress reference and current steering target in field units.
    /// @details lookahead may be a gate, recovery point or tangent extension;
    /// interpolated theta is not a steering or final-heading command.
    Pose reference, lookahead;
    double progress = 0;       ///< Earned arc length in inches; never decreases within a request.
    double length = 0;         ///< Prepared polyline arc length in inches; zero for a singleton.
    double crossTrack = 0;     ///< Unsigned inches to earned reference, not global nearest-path error.
    double curvature = 0;     ///< Steering chord curvature, clockwise inverse inches.
    double targetRPM = 0;      ///< Normalized floating center request in nonnegative motor RPM.
    double measuredSpeed = 0;  ///< Pose displacement / preceding dt, inches/s; first sample is zero.
    double angularVelocity = 0; ///< Floating wheel-pair yaw request, clockwise radians/s.
  };

 private:
  // Profiles are owned copies shared by legacy and path operations. setPath()
  // clears velocity/acceleration; construction forces both final velocities to zero.
  // Linear path distances are inches and linear profile output is motor RPM;
  // path acquisition supplies half-track arc inches to the angular profile.
  MotionProfile linearProfile;
  MotionProfile angularProfile;

  double lookaheadDistance;  // inches along the path, independent of sampling
  /// @brief Prepared consecutive-distinct XY samples; final theta is retained.
  std::vector<Pose> points;
  /// @brief Parallel cumulative inches and backward-braked center-RPM ceilings.
  /// @details Prepared lengths match points; distances starts at zero and strictly
  /// increases after duplicate collapse. The final speed ceiling is always zero.
  std::vector<double> distances, speedCaps;
  size_t segment = 0; ///< Active contiguous edge index; singleton requests retain zero.
  Status pathStatus = Status::IDLE; ///< Current direct-controller lifecycle state.
  Diagnostics diagnostics; ///< Reset on preparation; refreshed by active updates.
  Pose previousPose; ///< Last accepted update pose; usable only when hasPreviousPose is true.
  bool hasPreviousPose = false; ///< First sample cannot witness a gate or earn settlement.
  bool previouslyArrived = false; ///< Both observations must qualify before an interval earns dwell.
  /// @brief Absolute progress ceiling: one initial lookahead plus twice pose travel.
  /// @details Clamped to path length; neither corners nor recovery replenish it.
  double progressAllowance = 0;
  bool recoveringProgress = false; ///< Latches a fixed recovery target until arrival or earned branch change.
  bool acquiringHeading = false; ///< Latches stationary turning until the representable forward cone is reached.
  Pose recoveryReference; ///< Fixed field-space recovery target; does not itself authorize progress.
  double recoveryTolerance = 0; ///< Internal recovery arrival radius in inches, separate from final tolerance.
  double settledSeconds = 0; ///< Continuous sampled endpoint dwell, reset by motion or departure.
  /// @brief Experimental commanded lateral acceleration ceiling in inches/s^2.
  /// @todo Measure traction/current and tune this assumption on the actual robot;
  /// it provides neither physical safety margin nor a measured adhesion limit.
  static constexpr double maxLateralAcceleration = 30;

  /// @brief Check finite XY inches and heading degrees without geometric bounds.
  /// @param p Pose to inspect, including theta even when only XY steers.
  /// @return True exactly when all three components are finite.
  static bool finitePose(const Pose& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
  }

  /// @brief Latch a terminal reason and clear requested center/yaw diagnostics.
  /// @param reason Failure status selected by the detecting control operation.
  /// @return Zero left/right RPM requests; the caller must publish the stop.
  /// @note Other geometry diagnostics remain available for failure inspection.
  std::pair<double, double> fail(Status reason) {
    pathStatus = reason;
    diagnostics.targetRPM = diagnostics.angularVelocity = 0;
    return {0, 0};
  }

  double linearDeadband;  ///< Positive endpoint/progress tolerance in inches; robot tuning is caller-owned.
  double angularDeadband;  ///< Positive legacy-turn/acquisition tolerance in degrees.

  /// @brief Validate the legacy single-target domain, including its 250 ms dt bound.
  /// @param target Finite target XY inches and heading degrees.
  /// @param current Finite observed pose in the same field frame.
  /// @param dt Positive preceding interval in seconds, at most 0.25.
  /// @return Whether poses, profiles, tolerances and drive conversion are usable.
  /// @note Path update() has the stricter maxPathInterval scheduling envelope.
  bool validControlInput(const Pose& target, const Pose& current, double dt) const {
    return finitePose(target) && finitePose(current) && std::isfinite(dt) &&
        dt > 0 && dt <= 0.25 && linearProfile.isValid() && angularProfile.isValid() &&
        std::isfinite(linearDeadband) && linearDeadband > 0 &&
        std::isfinite(angularDeadband) && angularDeadband > 0 &&
        std::isfinite(DRIVE_WIDTH) && DRIVE_WIDTH > 0 &&
        std::isfinite(math::linearSpeed(1)) && math::linearSpeed(1) > 0;
  }

  /// @brief Proportionally limit a legacy wheel pair to linear/motor RPM ceilings.
  /// @param left Requested left motor RPM; may be negative.
  /// @param right Requested right motor RPM; may be negative.
  /// @return Finite scaled pair, or zero with CONTROL_FAILURE on nonfinite input.
  /// @pre validControlInput() has established valid positive profile limits.
  /// @note This does not apply path-specific center, angular or lateral limits.
  std::pair<double, double> boundedLegacy(double left, double right) {
    if (!std::isfinite(left) || !std::isfinite(right))
      return fail(Status::CONTROL_FAILURE);
    const double limit = std::min(double(MAX_RPM),linearProfile.getMaxVelocity());
    const double scale = std::max({1.0,std::abs(left)/limit,std::abs(right)/limit});
    return {left/scale,right/scale};
  }

 public:
  /// @brief Copy profiles and configure path/legacy controller tolerances.
  /// @param linearProfile Linear profile with motor-RPM limits and inch distance input.
  /// @param angularProfile Turning profile; path acquisition uses wheel arc inches.
  /// @param lookaheadOffset Path lookahead in inches (legacy name; not sample count).
  /// @param linearDeadband Positive endpoint/progress tolerance in inches.
  /// @param angularDeadband Positive legacy-turn/acquisition tolerance in degrees.
  /// @details Both copied profiles are forced to zero final velocity. Configuration
  /// validation occurs on control/preparation, not in this constructor.
  PurePursuit(MotionProfile linearProfile, MotionProfile angularProfile,
              int lookaheadOffset, double linearDeadband, double angularDeadband)
      : linearProfile(linearProfile), angularProfile(angularProfile) {
    this->linearProfile.setFinalVelocity(0);
    this->angularProfile.setFinalVelocity(0);
    this->lookaheadDistance = lookaheadOffset;
    this->linearDeadband = linearDeadband;
    this->angularDeadband = angularDeadband;
  }

  /// @brief Produce legacy single-target translation/heading wheel requests.
  /// @param target Target XY inches; theta is validated but does not guide go().
  /// @param current Observed pose, using clockwise degrees in the AON frame.
  /// @param dt Positive preceding interval in seconds, at most 0.25 (default 0.02).
  /// @return Left/right motor RPM; zero inside linearDeadband or on failure.
  /// @details Independently profiles distance and heading-error arc, then sums
  /// wheel requests. This is separate from the prepared path lifecycle, contiguous
  /// progress, settlement and path-specific safety ceilings. A zero result is
  /// not a path SUCCESS transition. Failure can update the shared status.
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

  /// @brief Produce legacy stationary turning requests toward target.theta.
  /// @param target Target pose; heading is clockwise degrees, XY only validated.
  /// @param current Observed pose in the same field frame.
  /// @param dt Positive preceding interval in seconds, at most 0.25 (default 0.02).
  /// @return Opposing left/right motor RPM, or zero in angularDeadband/on failure.
  /// @details Uses shortest wrapped degree error directly as angular-profile
  /// input, preserving legacy semantics. It does not own path final alignment
  /// or declare translation settlement; movement owns that separate stage.
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

  /// @brief Read direct-controller status without starting or advancing a request.
  /// @return Current status; movement-layer timeout/cancellation is tracked externally.
  Status status() const { return pathStatus; }
  /// @brief Convert a finite, normalized forward wheel pair to the actual motor
  /// representation. Independent truncation can increase center or turn RPM.
  /// Reduce only positive wheel magnitude to preserve the requested center and
  /// configured angular ceiling. Same-sign truncation changes the difference by
  /// less than 1RPM; opposite signs change the sum by less than 1RPM. Thus each
  /// center/angular constraint needs at most one decrement. The final lateral
  /// bound also constrains the larger wheel: |a| = u^2*|l^2-r^2|/(2*width),
  /// where u is inches/second per RPM. For a forward pair the larger signed
  /// wheel is >= |other|. Reducing it toward |other| decreases center, turn and
  /// lateral acceleration together, without reversing center or adding speed.
  /// @param left Normalized left equivalent motor RPM, possibly negative.
  /// @param right Normalized right equivalent motor RPM, possibly negative.
  /// @param angularLimit Finite nonnegative differential turn ceiling in RPM;
  /// abs(left-right)/2 must already respect it.
  /// @return Integer left/right RPM preserving nonincreased requested center
  /// speed and the angular/lateral ceilings; curvature can change at low speed.
  /// @note Uses ideal differential kinematics, not measured wheel motion. Work
  /// is fixed: at most one square root plus an integer-boundary rounding guard.
  /// @pre DRIVE_WIDTH and linearSpeed(1) are finite and strictly positive.
  /// @pre Wheel magnitudes are within motor int16 range, left+right >= 0,
  /// and the finite input pair already respects the finite angularLimit >= 0.
  static std::pair<int16_t,int16_t> motorCommands(double left, double right,
                                               double angularLimit) {
    int16_t l = static_cast<int16_t>(left), r = static_cast<int16_t>(right);
    if (l+r > left+right) {
      if (l >= r) --l;
      else --r;
    }
    if (std::abs(l-r) > 2*angularLimit) {
      if (l > r) --l;
      else --r;
    }
    const double unit = math::linearSpeed(1);
    const double squaredLimit = 2*DRIVE_WIDTH*maxLateralAcceleration/(unit*unit);
    int16_t& larger = l >= r ? l : r;
    const double other = l >= r ? r : l;
    if (double(larger)*larger-other*other > squaredLimit) {
      larger = static_cast<int16_t>(std::floor(std::sqrt(other*other+squaredLimit)));
      // No physical margin: this one-step guard covers sqrt rounding at an
      // integer boundary. All squares are evaluated in double, not int16/int.
      if (double(larger)*larger-other*other > squaredLimit) --larger;
    }
    return {l,r};
  }
  /// @brief Inspect the latest snapshot under caller-owned serialization.
  /// @return Borrowed internal diagnostics, invalid as a stable snapshot after mutation.
  const Diagnostics& getDiagnostics() const { return diagnostics; }
  /// @brief Retrieve the prepared terminal heading for movement-layer alignment.
  /// @return Last retained theta in clockwise degrees, or zero with no stored path.
  /// @pre Check successful preparation before interpreting partially prepared data.
  double finalHeading() const { return points.empty() ? 0 : points.back().theta; }

  /// @brief Copy, validate and reset an explicit ordered path request.
  /// @param path One to 4096 finite poses, XY inches with abs(X/Y) <= 1e6;
  /// intermediate theta is ignored by steering, final theta is retained unchanged.
  /// @param caps Optional same-length center-speed ceilings in equivalent motor
  /// RPM, positive except optional terminal zero; empty uses the linear maximum.
  /// @param reversed Must be false; reverse requests are explicitly unsupported.
  /// @return RUNNING on successful preparation, otherwise a latched failure.
  /// @details Consecutive XY within 1e-9 inches collapse to the latest pose and
  /// strictest cap. Singleton paths request endpoint settlement. Final cap is
  /// forced to zero and a backward v^2=u^2+2*a*s envelope propagates restrictions,
  /// using half the configured deceleration after conversion to inches/s^2.
  /// Allocations occur here; rejected preparation may retain partial storage.
  /// Explicitly call again for every new request, including after terminal status.
  /// @pre No concurrent instance access; caller owns stopping before replacement.
  /// @note This accepts generic caps without decoding any exporter metadata.
  Status setPath(const std::vector<Pose>& path,
                 const std::vector<double>& caps = {}, bool reversed = false) {
    points.clear(); distances.clear(); speedCaps.clear();
    diagnostics = Diagnostics{};
    segment = 0; settledSeconds = 0; hasPreviousPose = false; previouslyArrived = false;
    progressAllowance = lookaheadDistance;
    recoveringProgress = false;
    acquiringHeading = false;
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

  /// @brief Advance prepared translation using one pose sample, without allocation/I/O.
  /// @param current Finite field pose: XY inches and clockwise heading degrees.
  /// @param dt Positive measured preceding command/sample interval in seconds,
  /// at most maxPathInterval; default 0.02 supports direct callers.
  /// @return Floating left/right motor RPM; terminal/inactive states return zero.
  /// @details Progress is monotonic and limited by ordered gate witnesses and
  /// measured displacement allowance. Steering uses interpolated lookahead,
  /// recovery and fold limits before clockwise differential wheel kinematics.
  /// SUCCESS requires remaining arc and endpoint error <=linearDeadband and
  /// continuous 150 ms sampled dwell at <=1 inch/s and <=5 degrees/s. First pose
  /// establishes a baseline and earns no dwell. Final heading is not aligned.
  /// @pre Call setPath() successfully first; serialize all instance access.
  /// @note Caller owns pose freshness/accuracy, deadline/cancellation, integer
  /// conversion via motorCommands(), command publication and terminal stop.
  /// A finite freshly supplied pose does not prove sensor accuracy or liveness.
  /// @todo Validate the 150 ms/1 inch/s/5 degrees/s settlement assumptions and
  /// geometry/numerical thresholds against calibrated physical measurements.
  std::pair<double, double> update(Pose current, double dt = 0.02) {
    if (pathStatus != Status::RUNNING) return {0, 0};
    if (!finitePose(current) || !std::isfinite(dt) || dt <= 0 || dt > maxPathInterval)
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
    const double minimumChord = std::max(1e-6,math::linearSpeed(1)*dt);
    // Recovery's half-chord speed ceiling must still permit one integer RPM
    // just outside its internal arrival ball, for any supported following hold.
    // This resolution affects only recovery geometry, never final arrival.
    const double recoveryResolution = 2*math::linearSpeed(1)*maxPathInterval;
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
          if (!recoveringProgress) {
            // Retreat along the gate normal, not merely along the incoming edge:
            // near reversals that edge can run almost parallel to the gate.
            // Sampling must not shrink a geometric corner's recovery ball. Walk
            // back over its contiguous incoming straight run, retaining every
            // waypoint (and its speed restriction) in the prepared path.
            size_t incoming = segment;
            while (incoming > 0) {
              const double priorLength = distances[incoming]-distances[incoming-1];
              const double px = (points[incoming].x-points[incoming-1].x)/priorLength;
              const double py = (points[incoming].y-points[incoming-1].y)/priorLength;
              if (px*ux+py*uy <= 1-1e-6) break;
              --incoming;
            }
            const double incomingLength = distances[segment+1]-distances[incoming];
            // Two recovery resolution steps leave a representable arrival ball even
            // for a genuinely short run. This only chooses a recovery target;
            // it never changes progress, a gate witness or final arrival bounds.
            const double retreat = std::max(2*recoveryResolution,
                std::min(lookaheadDistance,incomingLength)*0.5);
            const double norm = std::hypot(nx,ny);
            gateRecoveryReference = Pose(b.x-nx/norm*retreat,b.y-ny/norm*retreat);
            // This arrival ball lies strictly before the gate (radius <= retreat/2).
            gateRecoveryTolerance = std::min(std::max(linearDeadband,recoveryResolution),retreat*0.5);
          }
        }
        if (distances[segment+1] > progressLimit && after >= -1e-9)
          progressLimited = true;
        if (!canSettle || moved <= 1e-9 ||
            distances[segment+1] > progressLimit ||
            distances[segment+1]-diagnostics.progress > lookaheadDistance) break;
        if (!ordered) break;
        lastCrossing = crossing;
        diagnostics.progress = distances[segment+1];
        // An earned crossing changes the geometric branch. A recovery point
        // for the old branch is obsolete; collinear samples alone do not.
        recoveringProgress = false;
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
      const bool arrived = canSettle && diagnostics.measuredSpeed <= 1 && angularSpeed <= 5;
      // The entering interval may start outside the arrival region. As in the
      // movement-owned heading stage, credit only consecutive qualified samples.
      settledSeconds = arrived && previouslyArrived ? settledSeconds + dt : 0;
      previouslyArrived = arrived;
      if (settledSeconds >= 0.15) pathStatus = Status::SUCCESS;
      return {0, 0};
    }
    settledSeconds = 0; previouslyArrived = false;
    // Supported active-path resolution: the half-chord recovery ceiling must
    // permit at least one integer RPM until its arrival ball is reached. This
    // sufficient experimental envelope is not a claim that smaller tolerances
    // are physically impossible. Already sampled arrival above still settles.
    if (linearDeadband < recoveryResolution)
      return fail(Status::UNSUPPORTED_CONFIGURATION);
    double targetDistance = std::min(diagnostics.length,
        diagnostics.progress + lookaheadDistance);
    // Bound the horizon at folds relative to both the active tangent and each
    // preceding future tangent. Positive active-direction projections alone can
    // hide a reversal between two future edges. Retain ordinary corner cutting.
    size_t nextGate = points.size();
    double gateNX = 0, gateNY = 0;
    if (segment+1 < points.size()) {
      const double length = distances[segment+1]-distances[segment];
      const double ux = (points[segment+1].x-points[segment].x)/length;
      const double uy = (points[segment+1].y-points[segment].y)/length;
      double previousX = ux, previousY = uy;
      for (size_t i=segment+1; i+1 < points.size() && distances[i]<targetDistance; ++i) {
        const double nextLength = distances[i+1]-distances[i];
        const double vx = (points[i+1].x-points[i].x)/nextLength;
        const double vy = (points[i+1].y-points[i].y)/nextLength;
        const double dot = previousX*vx+previousY*vy;
        if (nextGate == points.size() && dot <= 1-1e-6) {
          nextGate = i;
          gateNX = previousX+vx; gateNY = previousY+vy;
          if (gateNX*gateNX+gateNY*gateNY <= 1e-12) {
            gateNX = previousX; gateNY = previousY;
          }
        }
        if (ux*vx+uy*vy < -1e-6 || dot < -1e-6) { targetDistance=distances[i]; break; }
        previousX = vx; previousY = vy;
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
    // Several non-reversing future turns can still put the horizon back before
    // the first outstanding gate. Aim at that gate instead of an unreachable
    // target through it. This earns no progress; the directed witness is unchanged.
    if (nextGate < points.size() && targetDistance > distances[nextGate] &&
        (diagnostics.lookahead.x-points[nextGate].x)*gateNX+
        (diagnostics.lookahead.y-points[nextGate].y)*gateNY < -1e-9)
      diagnostics.lookahead = points[nextGate];
    // Recover to a fixed earned reference before looking forward again. Latch
    // the target until arrival: alternating between acquisition and lookahead
    // as a projection touches its allowance can otherwise strand the controller
    // rotating between two targets without earning any further displacement.
    if ((progressLimited || missedGate) && !recoveringProgress) {
      recoveryReference = missedGate ? gateRecoveryReference : diagnostics.reference;
      recoveryTolerance = missedGate ? gateRecoveryTolerance :
          std::max(recoveryResolution,std::min(linearDeadband,lookaheadDistance*0.25));
      recoveringProgress = true;
    }
    if (recoveringProgress) {
      if (current.distanceTo(recoveryReference) <= recoveryTolerance) recoveringProgress = false;
      else diagnostics.lookahead = recoveryReference;
    }
    // An active corner target closer than one integer-RPM travel step can
    // produce opposing integer wheels with no center motion, even though its
    // floating curvature is finite. Recover on the incoming tangent, crossing
    // the existing gate by measured displacement rather than minting progress.
    if (current.distanceTo(diagnostics.lookahead) <= minimumChord &&
        segment+1 < points.size()) {
      diagnostics.lookahead = points[segment+1];
      if (current.distanceTo(diagnostics.lookahead) <= minimumChord) {
        if (segment+2 < points.size()) {
          const double length = distances[segment+1]-distances[segment];
          diagnostics.lookahead = points[segment+1] +
              (points[segment+1]-points[segment])*(lookaheadDistance/length);
        } else diagnostics.lookahead = diagnostics.reference;
      }
    }
    const double dx = diagnostics.lookahead.x-current.x;
    const double dy = diagnostics.lookahead.y-current.y;
    const double heading = std::remainder(current.theta,360.0)*M_PI/180;
    // Rotate the field-space chord into robot axes. Positive lateral is right;
    // k=2*lateral/chord^2 is clockwise inverse inches in this frame.
    const double lateral = -std::sin(heading)*dx + std::cos(heading)*dy;
    const double forward = std::cos(heading)*dx + std::sin(heading)*dy;
    const double chordSquared = dx*dx+dy*dy;
    if (chordSquared <= 1e-12) return fail(Status::CONTROL_FAILURE);
    diagnostics.curvature = 2*lateral/chordSquared;
    double rpm = linearProfile.update(std::max(remaining,endError), dt);
    const double unit = math::linearSpeed(1);
    double cap = linearProfile.getMaxVelocity();
    // A latched recovery target is an arrival task, not a forward horizon.
    // Remaining path length cannot brake for it. Reserve half its current
    // chord for the longest supported following command hold (50 ms); the
    // measured dt describes the preceding hold and cannot predict that hold.
    // Later cosine, wheel and integer-pair limits can only reduce translation.
    if (recoveringProgress)
      cap = std::min(cap,current.distanceTo(recoveryReference)/(2*maxPathInterval*unit));
    if (segment + 1 < points.size()) {
      const double decel = math::linearSpeed(linearProfile.getMaxDeceleration()*0.5);
      // Projected end progress does not imply arrival: retain braking distance
      // to the actual endpoint while recovering a lateral offset or overshoot.
      const double brakingDistance = segment+2 == points.size() ?
          std::max(distances[segment+1]-diagnostics.progress,endError) :
          distances[segment+1]-diagnostics.progress;
      cap = std::min({cap,speedCaps[segment], std::sqrt(
          std::pow(speedCaps[segment+1]*unit,2) +
          2*decel*brakingDistance)/unit});
    }
    // SmartMotor truncates to integer RPM. Reject an unrepresentable active
    // translation ceiling instead of raising it or running forever at zero.
    // Arrival above takes precedence, including singleton and tiny end segments.
    if (std::min({cap,double(MAX_RPM),linearProfile.getMaxVelocity()}) < 1)
      return fail(Status::UNSUPPORTED_CONFIGURATION);
    // Ideal lateral acceleration is v^2*abs(k); convert its inch/s ceiling to
    // motor RPM before combining it with profile and waypoint restrictions.
    const double curveCap = std::sqrt(maxLateralAcceleration /
        std::max(std::abs(diagnostics.curvature),1e-9))/unit;
    const double error = std::atan2(lateral,forward);
    rpm = std::min({rpm, cap, curveCap, double(MAX_RPM)}) *
          std::max(0.0,std::cos(error));
    // Differential kinematics: wheel RPM = center RPM*(1 +/- k*width/2).
    // Proportional normalization retains the floating ratio; integer transport
    // can reduce coupled constraints further and change that ratio.
    double turnRPM = rpm*diagnostics.curvature*DRIVE_WIDTH/2;
    const double wheelLimit = std::min(double(MAX_RPM),linearProfile.getMaxVelocity());
    const double trackingScale = std::max({1.0,std::abs(rpm+turnRPM)/wheelLimit,
        std::abs(rpm-turnRPM)/wheelLimit,
        std::abs(turnRPM)/angularProfile.getMaxVelocity()});
    const auto trackingCommand = motorCommands((rpm+turnRPM)/trackingScale,
        (rpm-turnRPM)/trackingScale,angularProfile.getMaxVelocity());
    // Forward-only tracking also needs acquisition when both final wheel
    // commands would truncate to zero in SmartMotor's integer-RPM transport.
    // Test after all wheel/turn limits, without raising either requested speed.
    // Latch to the existing angular tolerance: releasing at the first nonzero
    // wheel can chatter at the representability boundary near 90 degrees.
    // A permissive tolerance must still leave the target in the forward cone
    // (forward >= |lateral|), keeping cosine attenuation away from its zero.
    // One +/-1 RPM turn step is the smallest transported heading increment.
    // A smaller requested tolerance must not latch an unachievable heading.
    const double acquisitionTolerance = std::min(std::max(
        angularDeadband*M_PI/180,2*unit*dt/DRIVE_WIDTH),M_PI/4);
    if (std::abs(error) >= M_PI/2 ||
        (std::abs(error) > acquisitionTolerance &&
         trackingCommand.first == 0 && trackingCommand.second == 0))
      acquiringHeading = true;
    if (std::abs(error) <= acquisitionTolerance) acquiringHeading = false;
    if (acquiringHeading) {
      if (std::min(wheelLimit,angularProfile.getMaxVelocity()) < 1)
        return fail(Status::UNSUPPORTED_CONFIGURATION);
      rpm = 0;
      turnRPM = std::copysign(angularProfile.update(
          std::abs(error)*DRIVE_WIDTH/2,dt), error);
    } else { angularProfile.setVelocity(0); angularProfile.setAccel(0); }
    double left = rpm+turnRPM, right = rpm-turnRPM;
    const double scale = std::max({1.0,std::abs(left)/wheelLimit,std::abs(right)/wheelLimit,
        std::abs(turnRPM)/angularProfile.getMaxVelocity()});
    left /= scale; right /= scale;
    if (!std::isfinite(left) || !std::isfinite(right) || rpm < 0)
      return fail(Status::CONTROL_FAILURE);
    diagnostics.targetRPM = rpm/scale;
    diagnostics.angularVelocity = (left-right)*unit/DRIVE_WIDTH;
    // Feed normalized floating center speed back to avoid cap windup while
    // retaining sub-RPM profile accumulation. Integer transport may reduce it.
    linearProfile.setVelocity(diagnostics.targetRPM);
    return {left,right};
  }

  /// @brief Prepare only from IDLE, then advance the existing compatibility request.
  /// @param path Path used only on the first IDLE call; later arguments do not
  /// replace the active or terminal request. Use setPath() explicitly to restart.
  /// @param current Observed field pose, XY inches and clockwise heading degrees.
  /// @param dt Positive preceding interval in seconds, at most maxPathInterval.
  /// @return update() left/right motor RPM, including zero for terminal status.
  /// @pre Caller owns serialization and the same publication/lifecycle duties as update().
  std::pair<double, double> follow(const std::vector<Pose>& path, Pose current,
                                  double dt = 0.02) {
    if (pathStatus == Status::IDLE) setPath(path);
    return update(current,dt);
  }
};

}  // namespace aon
