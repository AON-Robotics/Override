#pragma once

#include "pros/motors.hpp"
#include "../odometry/odometry.hpp"
#include "../controls/smart_motor.hpp"
#include "../controls/pid/pid.hpp"
#include "../controls/s-curve-profile.hpp"
#include "../math/timer.hpp"
#include "../controls/pure-pursuit.hpp"
#include <cfloat>
#include <atomic>
#include <mutex>
#include "pros/rtos.hpp"


namespace aon {


class Drivetrain {
  public:

  /// @brief This struct holds the factors by which inputs will be scaled when driving during `turbo` and `!turbo` states
  struct SpeedFactors {
    double forwardNoTurbo = 0.5;
    double sidewaysNoTurbo = 0.5;
    double turnNoTurbo = 0.5;

    double forwardTurbo = 1;
    double sidewaysTurbo = 1;
    double turnTurbo = 1;

    SpeedFactors(
      double forwardNoTurbo = 0.5,
      double sidewaysNoTurbo = 0.5,
      double turnNoTurbo = 0.5,
      double forwardTurbo = 1,
      double sidewaysTurbo = 1,
      double turnTurbo = 1
    ) : forwardNoTurbo(forwardNoTurbo),
        sidewaysNoTurbo(sidewaysNoTurbo),
        turnNoTurbo(turnNoTurbo),
        forwardTurbo(forwardTurbo),
        sidewaysTurbo(sidewaysTurbo),
        turnTurbo(turnTurbo) {}
  };

  protected:

  std::unique_ptr<Odometry> odometry;
  bool turbo = false;

  /// @brief This applies only while using curvature drive to allow for turning without forward motion. Any forward motion below this will cause curvature drive to behave like arcade.
  static constexpr double QUICK_TURN_THRESHOLD = 0.05 * MAX_RPM;

  SpeedFactors speedFactors;

  std::unique_ptr<MotionProfile> xProfile;
  std::unique_ptr<MotionProfile> yProfile;
  std::unique_ptr<MotionProfile> thetaProfile;

  /// @brief Revocation epoch captured by each admitted synchronous request.
  /// @details Incremented before waiting on publication so old requests lose
  /// command authority promptly. Equality assumes fewer than 2^32 revocations
  /// during one request; this counter is independent of the timing clocks.
  std::atomic<uint32_t> pathCancelGeneration{0};
  /// @brief Persistent admission gate; direct/manual construction starts enabled.
  /// @note Gates followPath only, including its optional heading stage.
  std::atomic<bool> pathEnabled{true};
  /// @brief Single followPath owner across preparation, both stages and cleanup.
  std::atomic<bool> pathActive{false};
  /// @brief Last admitted request's published state; overlap rejection leaves it alone.
  std::atomic<PurePursuit::Status> pathStatus{PurePursuit::Status::IDLE};

  /// @brief Serializes revocation barriers, motor publication and terminal stops.
  /// @details Lock order: drivetrain command mutex, then an individual motor-group mutex.
  /// Hold only for generation/state changes and a finite set of motor writes;
  /// never across pose reads, controller computation, delays or user callbacks.
  /// Kernel scheduling and motor I/O still prevent a hard real-time guarantee.
  pros::Mutex pathCommandMutex;
  /// @brief Transport-failure latch protected by pathCommandMutex.
  /// @details Cleared only on admission of a request with a current generation;
  /// a failure takes precedence over cancellation and success during cleanup.
  bool pathMotorFailure = false;

  /// @brief Attempt zero velocity through the drivetrain transport.
  /// @return True when the implementation reports successful stop publication.
  /// @pre pathCommandMutex is held by the caller.
  /// @warning Overrides must not reenter stop(), cancelPath(), publishPath(),
  /// setPathEnabled(), or acquire this mutex indirectly through tank(). The base
  /// fallback cannot detect transport failures; supported configurations must
  /// provide their validated transport contract. An API success is no physical
  /// proof that the robot stopped.
  virtual bool stopPathMotors() { this->motors(0); return true; }

  /// @brief Publish a wheel pair only while this request retains command authority.
  /// @param generation Request's captured revocation epoch.
  /// @param left Requested left wheel motor RPM; positive moves forward.
  /// @param right Requested right wheel motor RPM; positive moves forward.
  /// @return CANCELLED for revoked/disabled authority, CONTROL_FAILURE for a
  /// latched or newly reported transport failure, otherwise RUNNING.
  /// @details The gate check and transport share one mutex with cancellation.
  /// A failing transport may already have updated one motor group; terminal
  /// cleanup attempts zeros, but publication is not a hardware transaction.
  PurePursuit::Status publishPath(uint32_t generation, double left, double right) {
    std::lock_guard<pros::Mutex> lock(pathCommandMutex);
    if (!pathEnabled.load() || pathCancelGeneration.load() != generation)
      return PurePursuit::Status::CANCELLED;
    if (pathMotorFailure || !commandPath(left,right)) {
      pathMotorFailure = true;
      return PurePursuit::Status::CONTROL_FAILURE;
    }
    return PurePursuit::Status::RUNNING;
  }

  /// @brief Opt in only configurations with validated differential wheel kinematics.
  /// @return False by default; concrete supported drivetrains override this hook.
  virtual bool supportsPurePursuit() const { return false; }

  /// @brief Send a path wheel pair through the concrete drivetrain transport.
  /// @param left Requested left motor RPM, positive forward.
  /// @param right Requested right motor RPM, positive forward.
  /// @return Transport success; the base tank() fallback cannot detect errors.
  /// @pre pathCommandMutex is held; obey stopPathMotors()'s non-reentry rule.
  /// @note Concrete adapters own conversion and error reporting. One side can
  /// succeed before the other fails; callers must handle partial publication.
  virtual bool commandPath(double left, double right) {
    tank(left,right);
    return true;
  }
  
  public:

  /// @brief Construct a drivetrain and initialize its pose when odometry exists.
  /// @param pose Initial X/Y in inches and clockwise heading in degrees.
  /// @param odom Owned pose provider; null is accepted for fail-closed path rejection.
  /// @param speedFactors Joystick scaling configuration.
  /// @param xProfile Owned lateral motion profile used by other motion APIs.
  /// @param yProfile Owned forward profile; required by followPath().
  /// @param thetaProfile Owned wheel-equivalent turn profile; required by followPath().
  /// @warning A null odom is safe for path admission/freshness checks only; legacy
  /// pose accessors and initialize() still require a valid provider.
  Drivetrain(Pose pose, std::unique_ptr<Odometry> odom, SpeedFactors speedFactors, 
             std::unique_ptr<MotionProfile> xProfile, std::unique_ptr<MotionProfile> yProfile, std::unique_ptr<MotionProfile> thetaProfile): 
             odometry(std::move(odom)), speedFactors(speedFactors),
             xProfile(std::move(xProfile)), yProfile(std::move(yProfile)), thetaProfile(std::move(thetaProfile)) {
    // Missing odometry must reach followPath's freshness rejection and stop
    // attempts without dereferencing a sensor during construction.
    if (odometry) setPose(pose);
  }

  enum DriveMode {
    TANK,
    ARCADE,
    SPLIT_ARCADE,
    CURVATURE,
    SPLIT_CURVATURE,
    HOLONOMIC,
  };

  // TODO: move all implementations to a dedicated cpp file

  /// @brief Starts the underlying odometry thread
  void initialize() { this->odometry->initialize(); }

  Pose getPose() { return this->odometry->getPose(); }
  double getX() { return this->odometry->getX(); }
  double getY() { return this->odometry->getY(); }
  double getTheta() { return this->odometry->getDegrees(); }

  void setPose(Pose p) { odometry->resetCurrent(p.x, p.y, p.theta); }
  void setX(double x) { odometry->SetPosition(x, getY()); }
  void setY(double y) { odometry->SetPosition(getX(), y); }
  void setTheta(double theta) { odometry->setDegrees(theta); }

  /// @brief Query the provider's pose freshness gate without dereferencing null.
  /// @return False with no odometry or when its existing freshness policy rejects.
  /// @note This does not acquire a pose atomically with the freshness check.
  bool hasFreshPose() { return odometry && odometry->hasFreshPose(); }
  /// @brief Query OTOS freshness under the provider's existing policy.
  /// @return False when odometry is absent or OTOS is not fresh.
  bool hasFreshOtos() { return odometry && odometry->hasFreshOtos(); }
  /// @brief Expose the provider's current pose-source identifier.
  /// @return Provider-defined source value, or 0 without odometry.
  int getPoseSource() { return odometry ? odometry->getPoseSource() : 0; }
  /// @brief Report whether the provider currently fuses IMU observations.
  /// @return False without odometry; otherwise the provider's fusion state.
  bool isImuFusing() { return odometry && odometry->isImuFusing(); }

  /// @brief Read the OTOS clockwise heading in degrees.
  /// @return The provider's OTOS heading, or PROS_ERR_F without odometry.
  /// @note Availability here is not evidence of OTOS freshness.
  double getOtosTheta() { return odometry ? odometry->getOtosDegrees() : PROS_ERR_F; }

  void resetPose(double x = 0.0, double y = 0.0, double theta = 0.0) {
    this->odometry->resetCurrent(x, y, theta);
  }


  bool isTurbo() { return this->turbo; }
  void setTurbo(bool turbo) { this->turbo = turbo; }
  void toggleTurbo() { this->turbo = !this->turbo; }

  /// @brief Moves all motors the same `rpm` to move forward
  /// @param rpm The speed in which to move all motors in \b rpm
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  void motors(const double &rpm = MAX_RPM, const int& delay = 0) {
    this->tank(rpm, rpm);
    if(delay == 0) return;
    pros::delay(delay);
    this->stop();
  }

  /// @brief Moves all motors the same `rpm` to move sideways
  /// @param rpm The speed in which to move all motors in \b rpm
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  virtual void sideways(const double &rpm = MAX_RPM, const int& delay = 0) {}

  /// @brief Moves all motors the same `rpm` to rotate clockwise
  /// @param rpm The speed in which to move all motors in \b rpm
  /// @param delay The amount of milliseconds between activation and deactivation, a delay of 0 will never deactivate the motors
  void rotate(const double &rpm = MAX_RPM, const int& delay = 0) {
    this->tank(rpm, -rpm);
    if(delay == 0) return;
    pros::delay(delay);
    this->stop();
  }

  /// @brief Drives the robot using tank control, mapping left and right inputs directly to each side of the drivetrain
  /// @param left The \b RPM to send to the left-side motors (positive is forward)
  /// @param right The \b RPM to send to the right-side motors (positive is forward)
  virtual void tank(const double &left, const double &right) = 0;

  /// @brief Drives the robot using arcade control, combining a forward and a turn input into left/right motor outputs
  /// @param forward The \b RPM to send to all motors for linear movement (positive is forward)
  /// @param turn The \b RPM to add/subtract from each side for rotational movement (positive is clockwise)
  void arcade(const double &forward, const double &turn) {
    double left = forward + turn;
    double right = forward - turn;

    // Normalize if either side exceeds MAX_RPM
    double maxVal = std::max(std::abs(left), std::abs(right));
    if (maxVal > MAX_RPM) {
      left  = (left  / maxVal) * MAX_RPM;
      right = (right / maxVal) * MAX_RPM;
    }

    this->tank(left, right);
  };

  /// @brief Drives the robot using curvature (cheesy drive) control, scaling the turn rate by the forward speed for smoother high-speed arcing
  /// @param forward The \b RPM to send to all motors for linear movement (positive is forward)
  /// @param turn The curvature input used to scale the rotational output relative to forward speed (positive is clockwise)
  void curvature(const double &forward, const double &turn) {
    bool quickTurn = std::abs(forward) < QUICK_TURN_THRESHOLD;

    double left, right;
    if (quickTurn) {
      // Fall back to arcade-style turning in place
      left  = forward + turn;
      right = forward - turn;
    } else {
      left  = forward + std::abs(forward) * turn;
      right = forward - std::abs(forward) * turn;
    }

    // Normalize if either side exceeds MAX_RPM
    double maxVal = std::max(std::abs(left), std::abs(right));
    if (maxVal > MAX_RPM) {
      left  = (left  / maxVal) * MAX_RPM;
      right = (right / maxVal) * MAX_RPM;
    }

    this->tank(left, right);
  };

  /// @brief Drives a holonomic (e.g. mecanum or X-drive) robot with independent forward, sideways, and rotational control
  /// @param forward The \b RPM to send to all motors for linear forward/backward movement (positive is forward)
  /// @param sideways The \b RPM to send to all motors for lateral strafe movement (positive is rightward)
  /// @param turn The \b RPM to send to all motors for rotational movement (positive is clockwise)
  virtual void holonomic(const double &forward, const double &sideways, const double &turn) {
    this->arcade(forward, turn);
    this->sideways(sideways);
  };

  /// @brief Drives the robot in the direction of the left joystick while turning it with the right joystick
  /// @param leftX The value of the left joystick on the x-axis in the range [-1, 1]
  /// @param leftY The value of the left joystick on the y-axis in the range [-1, 1]
  /// @param rightX The value of the right joystick on the x-axis in the range [-1, 1]
  /// @param rightY The value of the right joystick on the y-axis in the range [-1, 1]
  void drive(double leftX, double leftY, double rightX, double rightY, DriveMode mode = HOLONOMIC) {
    double left, right, forward, sideways, turn;

    // Determine movement scaling factors depending on turbo status
    double forwardFactor = this->isTurbo() ? speedFactors.forwardTurbo : speedFactors.forwardNoTurbo;
    double sidewaysFactor = this->isTurbo() ? speedFactors.sidewaysTurbo : speedFactors.sidewaysNoTurbo;
    double turnFactor = this->isTurbo() ? speedFactors.turnTurbo : speedFactors.turnNoTurbo;

    switch (mode) {
      case TANK:
        left = applySpeed(leftY, forwardFactor);
        right = applySpeed(rightY, forwardFactor);

        this->tank(left, right);
        break;

      case ARCADE:
        forward = applySpeed(leftY, forwardFactor);
        turn = applySpeed(leftX, turnFactor);

        this->arcade(forward, turn);
        break;

      case SPLIT_ARCADE:
        forward = applySpeed(leftY, forwardFactor);
        turn = applySpeed(rightX, turnFactor);

        this->arcade(forward, turn);
        break;

      case CURVATURE:
        forward = applySpeed(leftY, forwardFactor);
        turn = applySpeed(leftX, turnFactor);

        this->curvature(forward, turn);
        break;

      case SPLIT_CURVATURE:
        forward = applySpeed(leftY, forwardFactor);
        turn = applySpeed(rightX, turnFactor);

        this->curvature(forward, turn);
        break;

      case HOLONOMIC:
        forward = applySpeed(leftY, forwardFactor);
        sideways = applySpeed(leftX, sidewaysFactor);
        turn = applySpeed(rightX, turnFactor);

        this->holonomic(forward, sideways, turn);
        break;

      default:
        left = applySpeed(leftY, forwardFactor);
        right = applySpeed(rightY, forwardFactor);

        this->tank(left, right);
        break;
    }
  }

  /// @brief Revoke the current path generation and attempt zero motor commands.
  /// @details Waits for any publication already inside the command mutex, then
  /// attempts a stop through the concrete transport. Admission is unchanged;
  /// if enabled, a later explicit request may run. Terminal status is finalized by the
  /// follow task, not by this method.
  /// @note Blocking mutex/transport latency is cooperative, not bounded real time.
  /// @warning Raw/legacy motion writers must be serialized separately by callers.
  virtual void stop() {
    // Revoke admission before waiting so the publishing task cannot repeatedly
    // reacquire the mutex with further commands while a stop is pending.
    pathCancelGeneration.fetch_add(1);
    std::lock_guard<pros::Mutex> lock(pathCommandMutex);
    if (!stopPathMotors()) pathMotorFailure = true;
  }

  /// @brief Revoke a path and wait for the in-flight publication barrier.
  /// @details Task-safe cooperative cancellation: after return, a request carrying
  /// the revoked generation cannot publish another wheel pair. This waits for
  /// current transport calls; it does not wait for followPath() to return, write
  /// zeros itself, or prevent a newly requested path. The follow task attempts
  /// terminal zeros when it next observes cancellation; use stop() to request
  /// zeros at this barrier too.
  /// @note getPathStatus() may remain RUNNING until terminal cleanup completes.
  void cancelPath() {
    pathCancelGeneration.fetch_add(1);
    std::lock_guard<pros::Mutex> lock(pathCommandMutex);
  }

  /// @brief Persistently allow/reject followPath requests at the competition boundary.
  /// @param enabled True permits a new explicit request; false revokes and stops.
  /// @details Disabling revokes the active translation/heading generation before waiting,
  /// then attempts zeros under the publication barrier. Enabling permits a new
  /// explicit request; it never revives a cancelled request. Raw driver commands
  /// and legacy motion APIs remain caller-owned and are not gated by this flag.
  void setPathEnabled(bool enabled) {
    if (!enabled) {
      pathEnabled.store(false);
      pathCancelGeneration.fetch_add(1);
    }
    std::lock_guard<pros::Mutex> lock(pathCommandMutex);
    // Revoke again at the serialized boundary: an overlapping enable may have
    // admitted a request carrying the first revocation's generation while this
    // disable waited. That request must stay cancelled after a later enable.
    // The pre-lock revocation still interrupts earlier requests immediately.
    if (!enabled) pathCancelGeneration.fetch_add(1);
    pathEnabled.store(enabled);
    if (!enabled && !stopPathMotors()) pathMotorFailure = true;
  }
  /// @brief Read the last admitted request's atomic lifecycle status.
  /// @return IDLE initially, RUNNING during admission/execution/cleanup, then its
  /// terminal outcome. Overlap rejection does not replace the owner's status.
  /// @note This snapshot does not imply physical motion or synchronize trace reads;
  /// join/wait for followPath() to return before inspecting caller-owned capture.
  PurePursuit::Status getPathStatus() const { return pathStatus.load(); }

  /// @brief Configures the general settings for the motors
  /// @param brakeMode The braking paradigm we will use, usually `holding` for
  /// auton and `brake` for drivers
  /// @param gearset The gearbox the physical motors contain, they MUST be all
  /// the same
  /// @param slew The slew rate for the motors, if 0, slew rate is `inf`
  void configure(pros::MotorBrake brakeMode, pros::MotorGears gearset, double slew) {
    this->setBrakeMode(brakeMode);
    this->setGearset(gearset);
    this->setEncoderUnits(pros::MotorEncoderUnits::degrees);

    if(brakeMode == pros::MotorBrake::hold){
      this->setSlewRate(0);
    } else {
      this->setSlewRate(slew);
    }
  }

  /// @brief Moves the robot back and forth a set amount of times
  /// @param count The amount of jiggles to do
  /// @param rpm The revolutions per minute at which to do the jiggles
  /// @param delay The time each jiggle lasts
  void jiggle(int count = 3, double rpm = 125, int delay = 225){
    while(count--) {
      this->motors(rpm, delay);
      this->motors(-rpm, delay);
    }
  }

  /// @brief Sets the brake mode for all motors of the drivetrain
  /// @param brakeMode The new brake mode for the drivetrain
  virtual void setBrakeMode(pros::MotorBrake brakeMode) = 0;

  /// @brief Sets the gearset for all motors of the drivetrain
  /// @param gearset The new gearset for the drivetrain
  virtual void setGearset(pros::MotorGears gearset) = 0;

  /// @brief Sets the units for all encoders of the motors of the drivetrain
  /// @param units The new units for the drivetrain
  virtual void setEncoderUnits(pros::MotorEncoderUnits units) = 0;

  /// @brief Sets the slew rate for all motors of the drivetrain
  /// @param slew The new slew rate for the drivetrain
  virtual void setSlewRate(double slew) = 0;

  /// @brief Calculates average RPM forward
  /// @return The RPM of the motors with respect to the front of the robot
  virtual double getRPM() = 0;

  /// @brief Moves the robot a given distance (default forward)
  /// @param pid The PID used for the driving
  /// @param dist The distance to be moved in \b inches
  /// @param MAX_REVS The maximum RPM to send to the movement
  void drivePID(PID pid = PID(0.02, 0, 0), double dist = TILE_WIDTH, const double &MAX_REVS = 100.0) {
    if (!hasFreshPose() || dist == 0) { stop(); return; }
    const int sign = dist / abs(dist);  // Getting the direction of the movement
    dist = abs(dist);                   // Setting the magnitude to positive
    pid.Reset();
    
    Vector initialPos = odometry->getPosition();

    const double timeLimit = math::estimateTimetoTarget(dist, MAX_REVS);
    const double start_time = pros::micros() / 1E6;
    #define time (pros::micros() / 1E6) - start_time  // every time the variable is called it is recalculated automatically

    while ((odometry->getPosition() - initialPos).GetMagnitude() < dist &&
           time < 3 * timeLimit) {
      if (!hasFreshPose()) { stop(); break; }
      double currentDisplacement = (odometry->getPosition() - initialPos).GetMagnitude();
      double output = pid.Output(dist, currentDisplacement);

      pros::lcd::print(0, "Time Limit %.2f", timeLimit);
      pros::lcd::print(1, "Time: %.2f", time);
      pros::lcd::print(2, "Odometry Displacement %.2f", currentDisplacement);

      this->motors(sign * std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));
      pros::delay(10);
    }

    // Stop the motors
    this->stop();

    #undef time
  }

  /// @brief Turns the robot by a given angle (default clockwise)
  /// @param pid The PID to be used for the turn
  /// @param angle The angle to make the robot turn in \b degrees
  /// @param MAX_REVS The maximum RPM to send to the movement
  void turnPID(PID pid = PID(0.002, 0, 0), double angle = 90, const double &MAX_REVS = 50.0) {
    if (!hasFreshPose() || angle == 0) { stop(); return; }
    pid.Reset();
    const double startAngle = odometry->getDegrees();  // Angle relative to the start
    
    double timeLimit = math::getTimetoTurnDeg(std::abs(angle));

    const double startTime = pros::micros() / 1E6;
    #define time (pros::micros() / 1E6) - startTime

    while (time < 3 * timeLimit) {
      if (!hasFreshPose()) { stop(); break; }

      double traveledAngle = odometry->getDegrees() - startAngle;
      if (std::abs(angle - traveledAngle) < 1.5) break;
      
      double output = pid.Output(angle, traveledAngle);

      pros::lcd::print(0, "Time Limit %.2f", timeLimit);
      pros::lcd::print(1, "Time: %.2f", time);
      pros::lcd::print(2, "Gyroscope Displacement %.2f", traveledAngle);
      
      // Taking clockwise rotation as positive (to change this just flip the negative on the sign below)
      this->rotate(std::clamp(output * MAX_RPM, -MAX_REVS, MAX_REVS));

      pros::delay(10);
    }

    this->stop();

    #undef time
  }

  /// @brief S-graph motion profile for linear movement
  /// @param dist The distance to be moved in \b inches, positive values will move forward and negative values backwards
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  void driveProfiled(double dist = TILE_WIDTH, bool settle = true) {
    if (!hasFreshPose()) { stop(); return; }
    if (dist == 0) {
      if (settle) stop();
      return;
    }

    const int sign = dist / abs(dist);  // Direction of the movement
    dist = abs(dist);                   // Setting the magnitude to positive
    
    // Timeout determined experimentally
    const uint32_t timeoutMs = (dist / 3.0) * 1E3;
    Timer timer;
    timer.start(timeoutMs);
    
    double dt = 0.02;                   // (s)
    double currVelocity = 0;
    double traveledDist = 0;
    Vector startPos = odometry->getPosition();
    
    double now = pros::micros() / 1E6;
    double lastTime = now;
    
    this->yProfile->setVelocity(this->getRPM());
    this->yProfile->setFinalVelocity(settle ? 0 : 100);

    while (traveledDist < dist && !timer.isCompleted()) {
      if (!hasFreshPose()) { stop(); return; }
      const Pose currentPose = odometry->getPose();
      traveledDist = (Vector().SetPosition(currentPose.x, currentPose.y) - startPos).GetMagnitude();
      double remainingDist = dist - traveledDist;
      now = pros::micros() / 1E6;
      dt = now - lastTime;
      lastTime = now;

      // Debugging output
      pros::lcd::print(1, "Traveled %.2f / %.2f", traveledDist, dist);
      pros::c::controller_print(pros::controller_id_e_t::E_CONTROLLER_MASTER, 0, 0, "Trav %.2f / %.2f", traveledDist, dist);

      currVelocity = this->yProfile->update(remainingDist, dt);
      this->motors(sign * currVelocity);

      if (remainingDist <= 0) { break; }  // Overshoot prevention

      pros::delay(20);
    }

    if (settle) { this->stop(); }
  }

  /// @brief S-graph motion profile for linear movement
  /// @param dist The distance to be moved in \b inches, positive values will move right and negative values left
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  void strafeProfiled(double dist = TILE_WIDTH, bool settle = true) {
    if (!hasFreshPose()) { stop(); return; }
    if (dist == 0) {
      if (settle) stop();
      return;
    }

    const int sign = dist / abs(dist); // Getting the direction of the movement
    dist = abs(dist); // Setting the magnitude to positive

    // Timeout determined experimentally
    const uint32_t timeoutMs = (dist / 3.0) * 1E3;
    Timer timer;
    timer.start(timeoutMs);
    
    double dt = 0.02; // (s)
    double currVelocity = 0;
    double traveledDist = 0;
    Vector startPos = odometry->getPosition();

    double now = pros::micros() / 1E6;
    double lastTime = now;

    this->xProfile->setVelocity(this->getRPM());
    this->xProfile->setFinalVelocity(settle ? 0 : 100);

    while (traveledDist < dist && !timer.isCompleted()) {
      if (!hasFreshPose()) { stop(); return; }
      traveledDist = (odometry->getPosition() - startPos).GetMagnitude();
      // traveledDist += getSpeed(this->getRPM()) * dt; //# in case of odom failure

      double remainingDist = dist - traveledDist;
      now = pros::micros() / 1E6;
      dt = now - lastTime;
      lastTime = now;

      currVelocity = this->xProfile->update(remainingDist, dt);
      this->sideways(sign * currVelocity);

      if (remainingDist <= 0) { break; } // Overshoot prevention

      pros::delay(20);
    }

    if (settle) this->stop();
  }

  /// @brief S-graph motion profile for rotations
  /// @param angle The angle in \b degrees we wish to rotate the robot, positive is clockwise and negative is counter-clockwise
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  void turnProfiled(double angle = 90, bool settle = true) {
    if (!hasFreshPose()) { stop(); return; }
    if (angle == 0) {
      if (settle) stop();
      return;
    }

    const int sign = angle / abs(angle);  // Getting the direction of the movement
    angle = abs(angle);                   // Setting the magnitude to positive

    // Timeout determined experimentally
    const uint32_t timeoutMs = (std::sqrt(angle / 2)) * 1E3;
    Timer timer;
    timer.start(timeoutMs);

    const double circumference = DRIVE_WIDTH * M_PI;  // Of the robot's rotation, used in the condition to calculate the length of arc remaining
    double dt = 0.02;                     // (s)
    double currVelocity = 0;
    double currAngle;
    double traveledAngle = 0;

    double startAngle = odometry->getDegrees();

    double now;
    double lastTime = pros::micros() / 1E6;

    this->thetaProfile->setFinalVelocity(settle ? 0 : 50);

    while (traveledAngle < angle && !timer.isCompleted()) {
      if (!hasFreshPose()) { stop(); return; }
      currAngle = odometry->getDegrees();
      traveledAngle = abs(currAngle - startAngle);
      double remainingAngle = angle - traveledAngle;
      now = pros::micros() / 1E6;
      dt = now - lastTime;
      lastTime = now;
      
      // Debugging output
      pros::lcd::print(1, "Traveled: %.2f / %.2f", traveledAngle, angle);
      // pros::c::controller_print(pros::controller_id_e_t::E_CONTROLLER_MASTER, 0, 0, "Trav %.2f / %.2f", traveledAngle, angle);

      currVelocity = this->thetaProfile->update(circumference * (remainingAngle / 360.0), dt);
      this->rotate(sign * currVelocity);

      if (traveledAngle >= angle) { break; }  // Overshoot prevention

      pros::delay(20);
    }
    if (settle) this->stop();
  }

  /// @brief Moves the robot a given distance
  /// @param dist The distance to move in \b inches
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @details A positive `dist` makes the robot go forward while a negative `dist` makes the robot go backwards
  void move(const double &dist = TILE_WIDTH, bool settle = true) { // TODO: change this settle variable to a more direct `double finalVelocity = 0` or similar (ideally same concept just a better name)
    driveProfiled(dist, settle);
  }

  /// @brief Moves the robot a given distance
  /// @param dist The distance to move in \b inches
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @details A positive `dist` makes the robot go right while a negative `dist` makes the robot go left
  void strafe(const double &dist = TILE_WIDTH, bool settle = true) { // TODO: add optional enum to pick which controller to use (for now just PID and MotionProfile available)
    strafeProfiled(dist, settle);
  }

  /// @brief Turn the robot a given angle (default is clockwise)
  /// @param angle The angle to turn in \b degrees
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @details Clockwise is positive and counter-clockwise is negative
  void turn(const double &angle = 90, bool settle = true) {
    turnProfiled(angle, settle);
  }

  /// @brief Sets the max velocity for the drivetrains motion profile
  /// @param rpm The max velocity in \b RPM to pass to the motion profile
  void setMaxVelocity(const double &rpm) {
    this->yProfile->setMaxVelocity(rpm);
  }

  /// @brief Calculates the target velocity to send to the motors for smooth and
  /// precise movements using an S-curve profile.
  /// @param distance The remaining distance to the target in \b inches.
  /// @param dt The time elapsed since the last function call in \b seconds.
  /// @return The updated velocity in \b RPM.
  double updateProfile(const double &distance, const double &dt) {
    return this->yProfile->update(distance, dt);
  }

  /// @brief Makes the robot drive in an arc motion based on a given `radius`
  /// @param radius The radius of the arc of the motion in \b inches measured
  /// from the center of rotation of the robot to the reference point in the
  /// right when positive and in the left when negative
  /// @param speed The speed with which to drive in \b RPM (positive speed
  /// will go forward and negative speed will go backwards)
  /// @note A positive `radius` will cause a clockwise rotation, while a
  /// negative `radius` will cause a counter-clockwise rotation
  /// @see https://www.desmos.com/calculator/91cbd82e8b
  void driveInArc(double radius, const double &speed = 200) {
    if(radius == 0) return;
    const bool clockwise = radius > 0.0;
    radius = std::abs(radius);

    // Calculate wheel speeds based on center speed and arc geometry
    const double outerRatio =  (radius + (DRIVE_WIDTH / 2)) / radius;
    const double innerRatio = (radius - (DRIVE_WIDTH / 2)) / radius;
    const double outerSpeed = speed * outerRatio;
    const double innerSpeed = speed * innerRatio;

    double leftSpeed, rightSpeed;
    
    // Clockwise, more speed on the left
    if(clockwise) {
      leftSpeed = outerSpeed;
      rightSpeed = innerSpeed;
    }
    // Counter-clockwise, more speed on the right
    else {
      rightSpeed = outerSpeed;
      leftSpeed = innerSpeed;
    }

    this->tank(leftSpeed, rightSpeed);
  }

  /// @brief Makes the robot drive in an arc motion based on a given `radius`
  /// for a given `angle`
  /// @param radius The radius of the arc of the motion in \b inches measured
  /// from the center of rotation of the robot to the reference point in the
  /// right when positive and in the left when negative
  /// @param angle The angle of the arc we want to cover in \b degrees, a
  /// negative angle will cause the robot to go in reverse
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  /// @note A positive `radius` will cause a rotation with reference to a point
  /// to the right, while a negative `radius` will cause a rotation with
  /// reference to a point to the left
  /// @note A positive `angle` will cause a forward movement, while a negative
  /// `angle` will cause a backwards movement
  /// @see https://www.desmos.com/calculator/91cbd82e8b
  void driveAngleOfArc(const double &radius = DRIVE_WIDTH, const double &angle = 90, bool settle = true) {
    if (!hasFreshPose()) { stop(); return; }
    if (angle == 0) {
      if (settle) stop();
      return;
    }

    if (radius == 0) {
      turn(angle, settle);
      return;
    }
    const short sign = angle / std::abs(angle);
    const double distance = std::abs((2 * radius * M_PI) * (angle / 360));
    double midSpeed;
    double traveledDist = 0, remainingDist = distance;
    double dt = 0.02;
    double now = pros::micros() / 1E6;
    double lastTime = now;
    Pose previousPose = odometry->getPose();
    this->yProfile->setVelocity(this->getRPM());
    this->yProfile->setFinalVelocity(settle ? 0 : 100);

    // Timeout determined experimentally
    const uint32_t timeoutMs = (distance / 3.0) * 1E3;
    Timer timer;
    timer.start(timeoutMs);

    while (traveledDist < distance && !timer.isCompleted()) {
      if (!hasFreshPose()) { stop(); return; }
      const Pose currentPose = odometry->getPose();
      traveledDist += currentPose.distanceTo(previousPose);
      previousPose = currentPose;
      remainingDist = distance - traveledDist;
      if (remainingDist <= 0) break;
      now = pros::micros() / 1E6;
      dt = now - lastTime;
      midSpeed = this->yProfile->update(remainingDist, dt);
      lastTime = now;

      this->driveInArc(radius, sign * midSpeed);

      pros::delay(20);
    }

    if (settle) this->stop();
  }

  /// @brief Makes the robot drive in an arc motion to a specified point in the
  /// field
  /// @param x The x coordinate of the point we want to go to in \b meters
  /// @param y The y coordinate of the point we want to go to in \b meters
  /// @note Odometry must be working for global positioning on the field
  /// @see https://www.desmos.com/calculator/5abb373276
  void driveInArcTo(const double &x, const double &y) {
    if (!hasFreshPose()) { stop(); return; }
    // Get the current pose
    Vector position = odometry->getPosition();
    position.SetPosition(math::inchesToMeters(position.GetX()), math::inchesToMeters(position.GetY()));
    double heading = odometry->getDegrees();
    Vector target = Vector().SetPosition(x, y);

    // X is forward and Y is right, matching the pose heading convention.
    heading *=  M_PI / 180;

    // (heading - π/2) % π cannot be 0 because tan(heading) would not be defined
    const bool isTanHeadingDefined = std::fmod(heading - M_PI_2, M_PI) != 0;

    // Calculate slopes of tangent to circular path and secant that cuts through current point and desired point
    double m_t = isTanHeadingDefined ? std::tan(heading) : DBL_MAX;
    double m_s = (position.GetX() != x) ? (position.GetY() - y) / (position.GetX() - x) : DBL_MAX;

    // Avoid 0 division later by switching to a very small value if a 0 slope arises
    m_t = m_t == 0 ? DBL_MIN : m_t;
    m_s = m_s == 0 ? DBL_MIN : m_s;

    // Get midpoint of the secant
    Vector midpoint = Vector().SetPosition((position.GetX() + x) / 2, (position.GetY() + y) / 2);

    // Calculate the position of the center of the circular path
    double centerX = (midpoint.GetY() - position.GetY() - (position.GetX() / m_t) + (midpoint.GetX() / m_s)) / ((-1 / m_t) + (1 / m_s));
    double centerY = ((-1 / m_t) * (centerX - position.GetX())) + position.GetY();
    Vector center = Vector().SetPosition(centerX, centerY);

    // Get the radius using the pythagorean theorem
    double radius = std::hypot(position.GetX() - center.GetX(), position.GetY() - center.GetY());

    // Determine the angle with some geometry and trigonometry
    double angle = math::getAngleOfArc(position, target, center);

    // Use a projection to determine which way we are turning
    const double projectionStep = 0.001;
    const Vector projection = Vector().SetPosition(position.GetX() + (projectionStep * std::cos(heading)),
                                                  position.GetY() + (projectionStep * std::sin(heading)));
    const double projectionAngle = math::getAngleInCircle(projection, center);
    
    const double positionAngle = math::getAngleInCircle(position, center);
    const double targetAngle = math::getAngleInCircle(target, center);
    
    // If going clockwise, the center is to the right (positive radius) and to the left in a counter-clockwise movement (negative radius)
    const bool clockwise = (targetAngle < projectionAngle && projectionAngle < positionAngle) || (projectionAngle < positionAngle && positionAngle < targetAngle) || (positionAngle < targetAngle && targetAngle < projectionAngle);
    if (!clockwise) { radius *= -1; }
    
    // Check if we have to go the long way around
    const bool longWay = (math::getAngleOfArc(projection, target, center) > angle) || (positionAngle < targetAngle && targetAngle < projectionAngle);
    if (longWay) { angle = 360 - angle; }

    this->driveAngleOfArc(math::metersToInches(radius), angle);
  }

  /// @brief Turns the robot towards a specific direction
  /// @param x The x component of the point we wish to face
  /// @param y The y component of the point we wish to face
  /// @note Uses coordinate system from GPS in \b meters
  void turnToPoint(const double &x, const double &y) {
    if (!hasFreshPose()) { stop(); return; }
    Vector target = Vector().SetPosition(math::metersToInches(x),
                                          math::metersToInches(y));
    // Determine current position
    Pose current = odometry->getPose();
    // Do the movement
    turn(math::calculateTurn(target, current));
  }

  /// @brief Turns the robot to an absolute heading
  /// @param heading The target heading in \b degrees (same convention as odometry)
  /// @param settle If true, robot will stop after movement, if false, it will proceed at a constant speed
  void turnToHeading(const double &heading, bool settle = true) {
    if (!hasFreshPose()) { stop(); return; }
    const double delta = std::remainder(heading - odometry->getDegrees(), 360.0);
    turn(delta, settle);
  }
  

  /// @brief Goes to the target point
  /// @param x The x component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @param y The y component of the place where we want to go using the gps coordinate system (x, y) both need to be in the range (-1.8, 1.8)
  /// @note Uses coordinate system from GPS in \b meters
  void goToPoint(const double &x, const double &y) {
    if (!hasFreshPose()) { stop(); return; }
    Vector target = Vector().SetPosition(x, y);
    // Determine current position
    Vector current = odometry->getPosition();
    current.SetPosition(math::inchesToMeters(current.GetX()),
                        math::inchesToMeters(current.GetY()));
    // Do the movement
    const Vector targetInches = Vector().SetPosition(math::metersToInches(x),
                                                     math::metersToInches(y));
    turn(math::calculateTurn(targetInches, odometry->getPose()));
    move(math::findDistance(target, current));
  }

  /// @brief Goes to the target point
  /// @param pose The target pose
  /// @note Pose X/Y are inches; heading is clockwise degrees.
  virtual void goToPose(const Pose &pose) = 0;

  /// @brief Outcome and final diagnostics of one synchronous path request.
  /// @note Default-valued measurements do not imply a stage actually executed.
  struct PathResult {
    /// @brief Overall terminal outcome after stop attempts and precedence rules.
    PurePursuit::Status status = PurePursuit::Status::IDLE;
    /// @brief Translation controller outcome, preserved if heading later fails.
    /// @details If translation never reports a terminal status, cleanup fills
    /// this with the overall outcome. Overlap rejection leaves it IDLE.
    PurePursuit::Status translationStatus = PurePursuit::Status::IDLE;
    /// @brief True only for successful explicitly requested final heading settlement.
    bool headingCorrected = false;
    /// @brief Latest signed shortest heading error in clockwise degrees.
    /// @note Meaningful only after heading-stage evaluation; otherwise remains zero.
    double headingErrorDeg = 0;
    /// @brief Requested final clockwise heading in degrees, set after path preparation.
    /// @note This is a target, not an observed final heading or evidence of alignment.
    double finalHeading = 0;
    /// @brief Last translation geometry/control diagnostics; retained during heading.
    PurePursuit::Diagnostics diagnostics;
    /// @brief Milliseconds from post-admission budget start through terminal stop.
    /// @note Excludes the initial admission mutex wait; overlap rejection stays zero.
    uint32_t elapsedMs = 0;
    /// @brief Maximum timed translation controller.update() duration in microseconds.
    /// @note Excludes pose reads, diagnostics copying, heading PID, transport,
    /// mutex waits, trace recording, delays and terminal stop.
    uint32_t maxUpdateUs = 0;
  };

  /// @brief Identify the controller stage responsible for a trace observation.
  /// @details TRANSLATION follows geometry; HEADING optionally aligns the endpoint.
  enum class PathStage { TRANSLATION, HEADING };

  /// @brief Optional experimental observation of a path control iteration.
  /// @details Timestamps observe pose reads, not sensor acquisition. RPM values
  /// are requested commands, never measured motor speeds. No sample is promised
  /// for every early exit, handoff zero or terminal stop; terminal trace fields
  /// carry the cleanup outcome. Unsigned timing differences tolerate a single
  /// clock rollover when elapsed durations fit the 32-bit clock range.
  struct PathSample {
    /// @brief Low 32 bits of the microsecond clock after reading the pose.
    uint32_t timestampUs = 0;
    /// @brief Observed interval in microseconds; first sample of each stage is zero.
    uint32_t intervalUs = 0;
    /// @brief Microseconds around translation update, or heading timestep/PID/clamp.
    /// @note Excludes heading error/speed/settlement evaluation and trace recording.
    uint32_t computeUs = 0;
    /// @brief Microseconds around publishPath(), including mutex wait and transport.
    /// @note Excludes handoff zero, terminal stop and record construction.
    uint32_t publicationUs = 0;
    /// @brief Stage producing this observation.
    PathStage stage = PathStage::TRANSLATION;
    /// @brief Observed X/Y in inches and clockwise heading in degrees.
    Pose pose;
    /// @brief Last translation diagnostics; heading samples retain that geometry.
    PurePursuit::Diagnostics diagnostics;
    /// @brief Requested left/right floating motor RPM before transport conversion.
    double requestedLeftRPM = 0, requestedRightRPM = 0;
    /// @brief Converted integer left/right RPM requests, even if not published.
    int16_t leftRPM = 0, rightRPM = 0;
    /// @brief Controller terminal status or publication result for this sample.
    PurePursuit::Status status = PurePursuit::Status::IDLE;
    /// @brief True when the transport reported successful pair publication.
    /// @warning False can include a partial motor write; true is not evidence
    /// that the physical motors achieved the requested speed.
    bool published = false;
  };

  /// @brief Caller-owned bounded sequential capture with no control-loop allocation.
  /// @pre The trace and storage remain alive and unmodified until followPath returns.
  /// @warning Not thread-safe: inspect only after the synchronous call returns.
  /// @details Allocate outside the control task. capacity*sizeof(PathSample) is
  /// sample storage only, excluding this object and path preparation allocations.
  /// Full/zero-capacity capture drops observations without changing motor behavior.
  /// @todo Measure usable capture capacity, stack/heap headroom and capture timing
  /// on the actual V5 configuration before relying on physical-run diagnostics.
  struct PathTrace {
    /// @brief Writable storage of at least capacity samples; null allowed at zero capacity.
    PathSample* samples = nullptr;
    /// @brief Number of available sample slots, not bytes.
    size_t capacity = 0;
    /// @brief Number of stored samples; never exceeds capacity.
    size_t size = 0;
    /// @brief Saturating dropped-sample count and largest observed stage interval (us).
    uint32_t dropped = 0, maxIntervalUs = 0;
    /// @brief Overall terminal status written after terminal stop attempts.
    PurePursuit::Status status = PurePursuit::Status::IDLE;
    /// @brief Translation outcome copied from PathResult at cleanup.
    PurePursuit::Status translationStatus = PurePursuit::Status::IDLE;
    /// @brief Whether the terminal stop hook reported success, not physical rest.
    bool terminalStopSucceeded = false;

    /// @brief Clear capture counters/outcomes while retaining the supplied storage.
    /// @pre No concurrent reader/writer; does not clear existing sample bytes.
    void reset() {
      size = 0; dropped = 0; maxIntervalUs = 0;
      status = translationStatus = PurePursuit::Status::IDLE;
      terminalStopSucceeded = false;
    }
    /// @brief Append one observation or saturating-count it as dropped.
    /// @param sample Observation copied by value into the next available slot.
    /// @pre Exclusive access and valid storage for the declared capacity.
    /// @note Performs no allocation, formatting, clock read or device read.
    void record(const PathSample& sample) {
      if (samples && size < capacity) samples[size++] = sample;
      else if (dropped != UINT32_MAX) ++dropped;
    }
  };

  /// @brief Explicit opt-in Issue 33 endpoint-heading stage under the path owner.
  /// @details No robot tuning is implied. Configuration is copied before motion;
  /// the PID is reset and its timestep replaced with measured seconds each update.
  /// Null configuration skips alignment. All gains/integral bounds must be finite
  /// and nonnegative, with at least one positive gain, including when Ki is zero.
  /// @todo Validate heading tuning and real translation-to-heading handoff on V5;
  /// broader Issue 33 legacy movement integration remains separate work.
  struct HeadingCorrection {
    /// @brief PID mapping clockwise degree error to signed motor RPM.
    /// @note Integral activation and anti-windup bounds must be finite/nonnegative.
    PID controller;
    /// @brief Finite wheel RPM ceiling, >=1 and <=MAX_RPM and both profile ceilings.
    double maxRPM;
    /// @brief Finite absolute heading tolerance in degrees, strictly between 0 and 180.
    double toleranceDeg;
    /// @brief Required continuous settled dwell in milliseconds; must be nonzero.
    uint32_t settleMs;
    /// @brief Separate cooperative heading budget in milliseconds, greater than settleMs.
    uint32_t timeoutMs;
  };

  /// @brief Synchronously follow an experimental forward differential path.
  /// @param path Waypoint X/Y in inches and clockwise heading in degrees; nonempty,
  /// at most 4096 finite poses within preparation bounds. Intermediate headings
  /// do not steer geometry; the final heading is the optional alignment target.
  /// @param timeoutMs Translation/preparation budget in milliseconds, default 15000.
  /// Zero expires before an update. The initial admission mutex wait is excluded;
  /// preparation, translation handoff and terminal stop consume this budget.
  /// @param caps Optional per-waypoint center-speed ceilings in equivalent motor
  /// RPM, independent of export format. Empty uses profile ceilings; otherwise
  /// match path size, with finite positive entries except an optional final zero.
  /// @param reversed True is explicitly rejected as UNSUPPORTED_CONFIGURATION.
  /// @param headingCorrection Optional finite validated heading tuning; null skips
  /// alignment. Its subsequent timeoutMs budget begins after the zero handoff.
  /// @param trace Optional bounded capture; storage/configuration must remain alive
  /// and unmodified through this call. Nonzero capacity requires nonnull storage.
  /// @return Terminal PathResult: SUCCESS means settled translation, and settled
  /// heading when requested; CANCELLED means revoked/disabled; TIMEOUT means a
  /// checked stage budget expired; INVALID_PATH means rejected geometry/caps;
  /// UNSUPPORTED_CONFIGURATION means unsupported drivetrain/reverse/controller
  /// envelope; CONTROL_FAILURE covers invalid control/sensor/transport conditions
  /// or overlap. A rejected overlap returns immediately without stopping the owner,
  /// resetting trace, changing shared status, or running terminal cleanup.
  /// @details Translation uses 5-inch lookahead and 2.5-inch arrival/progress
  /// tolerances with 150ms motion settlement (<=1in/s and <=5deg/s). Optional
  /// heading retains endpoint position within 2.5 inches and requires heading
  /// tolerance plus those speed limits over measured intervals for settleMs.
  /// The first sample of each stage supplies a 10ms controller startup timestep
  /// but no measured interval or motion-settlement evidence. Later intervals use
  /// unsigned microsecond subtraction and must be positive and <=50ms.
  /// Budgets use unsigned millisecond subtraction across rollover, assuming each
  /// elapsed duration fits its 32-bit clock range. Checks are cooperative: mutex
  /// waits, preparation, scheduling and motor I/O can overrun deadlines; cleanup
  /// rejects SUCCESS if its active budget has expired when stop calls return.
  /// CONTROL_FAILURE (including transport failure) overrides cancellation;
  /// cancellation overrides other outcomes; a known expired budget replaces an otherwise successful result.
  /// Translation success is retained separately if heading or terminal cleanup fails.
  /// @pre Serialize other motion writers. Keep this drivetrain, odometry, profiles,
  /// input vectors, configuration and capture alive for the entire call.
  /// @warning Cancellation is task-safe; raw driver/legacy APIs are not gated.
  /// Do not destroy the object or forcibly remove its executing task: RAII owner
  /// release and terminal zero attempts require normal cooperative return.
  /// Software settlement/transport success is not measured physical accuracy.
  /// @todo Validate actual V5 timing, motor-stop behavior and competition lifecycle
  /// transitions physically. The Issue 8 export adapter remains separate work;
  /// students must understand, adapt, validate and attribute this reference code.
  PathResult followPath(const std::vector<Pose>& path, uint32_t timeoutMs = 15000,
                        const std::vector<double>& caps = {}, bool reversed = false,
                        const HeadingCorrection* headingCorrection = nullptr,
                        PathTrace* trace = nullptr) {
    using Status = PurePursuit::Status;
    PathResult result;
    const uint32_t generation = pathCancelGeneration.load();
    if (pathActive.exchange(true)) {
      result.status = Status::CONTROL_FAILURE;
      return result;
    }
    // Overlap is the intentional cleanup exception: it has no command ownership.
    /// @brief Release sole request ownership on every normal scoped return.
    /// @warning Forced task removal/destruction cannot rely on this cleanup.
    struct ReleaseOnExit {
      /// @brief Owning drivetrain; must outlive this synchronous request.
      Drivetrain* drive;
      /// @brief Permit another request only after terminal cleanup has returned.
      ~ReleaseOnExit() { drive->pathActive.store(false); }
    } releaseOnExit{this};
    if (trace) trace->reset(); // rejected overlapping requests never touch capture
    bool admitted = false;
    {
      std::lock_guard<pros::Mutex> lock(pathCommandMutex);
      admitted = pathEnabled.load();
      if (admitted && pathCancelGeneration.load() == generation) pathMotorFailure = false;
      pathStatus.store(Status::RUNNING);
    }
    const uint32_t started = pros::millis();
    uint32_t activeBudgetStarted = started;
    uint32_t activeBudgetMs = timeoutMs;
    /// @brief Finalize an owned request after serialized terminal zero attempts.
    /// @param status Proposed stage outcome, subject to failure/cancellation/deadline.
    /// @return Final result with elapsed time and trace terminal fields updated.
    /// @details Holds ownership until return; cleanup never revives a revoked epoch.
    auto finish = [&](Status status) {
      std::lock_guard<pros::Mutex> lock(pathCommandMutex);
      if (pathCancelGeneration.load() != generation && status != Status::CONTROL_FAILURE)
        status = Status::CANCELLED;
      // Publish terminal status only after both zero attempts have completed.
      // A failed terminal write must not be reported as settled success.
      const bool stopped = stopPathMotors();
      if (!stopped) pathMotorFailure = true;
      if (pathCancelGeneration.load() != generation && status != Status::CONTROL_FAILURE)
        status = Status::CANCELLED;
      if (pathMotorFailure) status = Status::CONTROL_FAILURE;
      const uint32_t finished = pros::millis();
      // Cooperative I/O may overrun a budget. Once writes return, a known
      // expired deadline must not become SUCCESS; stop errors/cancellation win.
      if (status == Status::SUCCESS &&
          uint32_t(finished-activeBudgetStarted) >= activeBudgetMs)
        status = Status::TIMEOUT;
      if (status != Status::SUCCESS) result.headingCorrected = false;
      result.status = status;
      if (result.translationStatus == Status::IDLE) result.translationStatus = status;
      result.elapsedMs = uint32_t(finished-started);
      pathStatus.store(status);
      if (trace) {
        trace->status = status;
        trace->translationStatus = result.translationStatus;
        trace->terminalStopSucceeded = stopped;
      }
      return result;
    };
    if (!admitted) return finish(Status::CANCELLED);
    if (trace && trace->capacity && !trace->samples) return finish(Status::CONTROL_FAILURE);
    if (!supportsPurePursuit() || reversed)
      return finish(Status::UNSUPPORTED_CONFIGURATION);
    if (!yProfile || !thetaProfile) return finish(Status::CONTROL_FAILURE);
    // Copy configuration before motion: the caller must not mutate it during
    // this call. Use finite anti-windup bounds, including when Ki is zero.
    HeadingCorrection correction{PID(0,0,0,0.01,0,0),0,0,0,0};
    if (headingCorrection) {
      correction = *headingCorrection;
      PID& pid = correction.controller;
      if (!std::isfinite(pid.GetKP()) || !std::isfinite(pid.GetKI()) ||
          !std::isfinite(pid.GetKD()) || pid.GetKP() < 0 || pid.GetKI() < 0 ||
          pid.GetKD() < 0 || pid.GetKP()+pid.GetKI()+pid.GetKD() <= 0 ||
          !std::isfinite(pid.GetStartIntegral()) || pid.GetStartIntegral() < 0 ||
          !std::isfinite(pid.GetAntiWindup()) || pid.GetAntiWindup() < 0 ||
          !std::isfinite(correction.maxRPM) || correction.maxRPM < 1 ||
          correction.maxRPM > std::min({double(MAX_RPM),yProfile->getMaxVelocity(),
                                       thetaProfile->getMaxVelocity()}) ||
          !std::isfinite(correction.toleranceDeg) || correction.toleranceDeg <= 0 ||
          correction.toleranceDeg >= 180 || correction.settleMs == 0 ||
          correction.timeoutMs <= correction.settleMs)
        return finish(Status::CONTROL_FAILURE);
      pid.Reset();
    }
    PurePursuit controller(*yProfile, *thetaProfile, 5, 2.5, 2.5);
    const Status prepared = controller.setPath(path,caps);
    if (prepared != Status::RUNNING) return finish(prepared);
    result.finalHeading = controller.finalHeading();
    /// @brief Construct a bounded capture observation without I/O or allocation.
    /// @param stage Stage owning this observation.
    /// @param pose Pose observed in inches/clockwise degrees.
    /// @param timestamp Observation clock in microseconds (low 32 bits).
    /// @param interval Measured preceding stage interval in microseconds, or zero.
    /// @param compute Timed controller work in microseconds, not full loop time.
    /// @param publication Timed publication/mutex duration in microseconds, or zero.
    /// @param left Requested floating left motor RPM.
    /// @param right Requested floating right motor RPM.
    /// @param status Controller or transport status at this observation.
    /// @param published Whether transport reported success for the pair.
    /// @note Conversion records an intended integer pair even when not published.
    const auto record = [&](PathStage stage, const Pose& pose, uint32_t timestamp,
                            uint32_t interval, uint32_t compute, uint32_t publication,
                            double left, double right, Status status, bool published) {
      if (!trace) return;
      PathSample sample;
      sample.timestampUs = timestamp; sample.intervalUs = interval;
      sample.computeUs = compute; sample.publicationUs = publication;
      sample.stage = stage; sample.pose = pose; sample.diagnostics = result.diagnostics;
      sample.requestedLeftRPM = left; sample.requestedRightRPM = right;
      const auto pair = PurePursuit::motorCommands(left,right,thetaProfile->getMaxVelocity());
      sample.leftRPM = pair.first; sample.rightRPM = pair.second;
      sample.status = status; sample.published = published;
      trace->record(sample);
    };
    uint32_t lastTime = uint32_t(pros::micros());
    bool first = true;
    while (true) {
      if (pathCancelGeneration.load() != generation) return finish(Status::CANCELLED);
      if (uint32_t(pros::millis()-started) >= timeoutMs) return finish(Status::TIMEOUT);
      if (!hasFreshPose()) return finish(Status::CONTROL_FAILURE);
      const Pose current = odometry->getPose();
      const uint32_t now = uint32_t(pros::micros());
      const uint32_t elapsedUs = first ? 0 : uint32_t(now-lastTime);
      const double dt = first ? 0.01 : elapsedUs/1e6;
      first = false; lastTime = now;
      if (trace) trace->maxIntervalUs = std::max(trace->maxIntervalUs,elapsedUs);
      const uint32_t computeStart = uint32_t(pros::micros());
      const auto output = controller.update(current,dt);
      const uint32_t computeUs = uint32_t(uint32_t(pros::micros())-computeStart);
      result.maxUpdateUs = std::max(result.maxUpdateUs,computeUs);
      result.diagnostics = controller.getDiagnostics();
      // Recheck asynchronous cancellation and deadline before publishing commands
      // or success. Zero commands alone are never evidence of completion.
      if (pathCancelGeneration.load() != generation) return finish(Status::CANCELLED);
      if (uint32_t(pros::millis()-started) >= timeoutMs) return finish(Status::TIMEOUT);
      if (controller.status() != Status::RUNNING) {
        record(PathStage::TRANSLATION,current,now,elapsedUs,computeUs,0,
               output.first,output.second,controller.status(),false);
        result.translationStatus = controller.status();
        if (controller.status() != Status::SUCCESS || !headingCorrection)
          return finish(controller.status());
        break;
      }
      const uint32_t publicationStart = trace ? uint32_t(pros::micros()) : 0;
      const Status published = publishPath(generation,output.first,output.second);
      record(PathStage::TRANSLATION,current,now,elapsedUs,computeUs,
             trace ? uint32_t(uint32_t(pros::micros())-publicationStart) : 0,
             output.first,output.second,published,published == Status::RUNNING);
      if (published != Status::RUNNING) return finish(published);
      pros::delay(10);
    }
    // Translation is verified settled. Stop before handing off to the single
    // heading owner, but retain ownership and generation across the handoff.
    Status published = publishPath(generation,0,0);
    if (published != Status::RUNNING) return finish(published);
    // The zero handoff still belongs to translation. Blocking publication must
    // not authorize heading correction after that stage's budget has expired.
    if (uint32_t(pros::millis()-started) >= timeoutMs) return finish(Status::TIMEOUT);
    const uint32_t correctionStarted = pros::millis();
    activeBudgetStarted = correctionStarted;
    activeBudgetMs = correction.timeoutMs;
    double settledSeconds = 0;
    bool previouslyArrived = false;
    Pose previousCorrectionPose;
    lastTime = uint32_t(pros::micros());
    first = true;
    while (true) {
      if (pathCancelGeneration.load() != generation) return finish(Status::CANCELLED);
      if (uint32_t(pros::millis()-correctionStarted) >= correction.timeoutMs)
        return finish(Status::TIMEOUT);
      if (!hasFreshPose()) return finish(Status::CONTROL_FAILURE);
      const Pose current = odometry->getPose();
      const uint32_t now = uint32_t(pros::micros());
      const bool measuredInterval = !first;
      const uint32_t elapsedUs = first ? 0 : uint32_t(now-lastTime);
      const double dt = first ? 0.01 : elapsedUs/1e6;
      first = false; lastTime = now;
      if (trace) trace->maxIntervalUs = std::max(trace->maxIntervalUs,elapsedUs);
      if (!std::isfinite(current.x) || !std::isfinite(current.y) ||
          !std::isfinite(current.theta) || dt <= 0 || dt > PurePursuit::maxPathInterval ||
          current.distanceTo(path.back()) > 2.5)
        return finish(Status::CONTROL_FAILURE);
      result.headingErrorDeg = std::remainder(std::remainder(result.finalHeading,360.0)-
          std::remainder(current.theta,360.0),360.0);
      const double normalizedHeading = std::remainder(current.theta,360.0);
      const double angularSpeed = measuredInterval ? std::abs(std::remainder(
          normalizedHeading-std::remainder(previousCorrectionPose.theta,360.0),360.0))/dt : 0;
      const double translationalSpeed = measuredInterval ?
          current.distanceTo(previousCorrectionPose)/dt : 0; // inches/second
      previousCorrectionPose = current;
      if (!std::isfinite(translationalSpeed)) return finish(Status::CONTROL_FAILURE);
      // Final handoff must retain translation's <=1in/s and <=5deg/s motion
      // settlement, not merely stay inside the position/heading tolerances.
      // The first sample establishes a baseline and supplies no dwell evidence.
      const bool arrived = measuredInterval &&
          std::abs(result.headingErrorDeg) <= correction.toleranceDeg &&
          translationalSpeed <= 1 && angularSpeed <= 5;
      // Only intervals whose endpoints were both in tolerance count as dwell.
      if (arrived && previouslyArrived) settledSeconds += elapsedUs/1e6;
      else settledSeconds = 0;
      previouslyArrived = arrived;
      const uint32_t computeStart = trace ? uint32_t(pros::micros()) : 0;
      correction.controller.SetKT(dt);
      double rpm = arrived ? 0 : correction.controller.Output(result.headingErrorDeg,0);
      if (!std::isfinite(rpm)) return finish(Status::CONTROL_FAILURE);
      rpm = std::clamp(rpm,-correction.maxRPM,correction.maxRPM);
      const uint32_t computeUs = trace ? uint32_t(uint32_t(pros::micros())-computeStart) : 0;
      if (pathCancelGeneration.load() != generation) return finish(Status::CANCELLED);
      if (uint32_t(pros::millis()-correctionStarted) >= correction.timeoutMs)
        return finish(Status::TIMEOUT);
      const uint32_t publicationStart = trace ? uint32_t(pros::micros()) : 0;
      published = publishPath(generation,rpm,-rpm);
      record(PathStage::HEADING,current,now,elapsedUs,computeUs,
             trace ? uint32_t(uint32_t(pros::micros())-publicationStart) : 0,
             rpm,-rpm,published,published == Status::RUNNING);
      if (published != Status::RUNNING) return finish(published);
      if (arrived && settledSeconds >= correction.settleMs/1000.0) {
        result.headingCorrected = true;
        return finish(Status::SUCCESS);
      }
      pros::delay(10);
    }
  }

  /// @brief Synchronous compatibility wrapper using followPath() defaults.
  /// @param path Waypoints in inches and clockwise degrees; followPath rules apply.
  /// @details Discards the explicit result and performs translation only. Inspect
  /// getPathStatus() after return, or call followPath() for diagnostics and optional
  /// heading alignment. Overlap rejection leaves the active request's status intact.
  /// @pre Same lifetime and motion-writer serialization requirements as followPath().
  virtual void follow(const std::vector<Pose>& path) { (void)followPath(path); }

  /// @brief Scales a joystick input to drivetrain motor intensity according to a percentage
  /// @param input The joystick input to be scaled
  /// @param percentage The percentage of the drivetrain's `MAX_RPM` to scale to
  /// @return The `input` scaled to the `MAX_RPM` of the drivetrain as per `percentage`
  static double applySpeed(const double& input, const double& percentage){
    return input * MAX_RPM * percentage;
  }
};

}  // namespace aon
