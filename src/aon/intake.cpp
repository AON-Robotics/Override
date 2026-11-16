#include "../../include/aon/intake/intake.hpp"

namespace aon {

#if USING_BIG_ROBOT

Intake::Intake(const std::initializer_list<std::int8_t>& elevatorPorts,
               const std::initializer_list<std::int8_t>& judgePorts,
               const std::initializer_list<std::int8_t>& liftPorts,
               const std::initializer_list<std::int8_t>& rollerPorts,
               char cartPistonsPort, int distanceSensorPort,
               int colorSensorPort, 
               char acceptSensorPort, char rejectSensorPort)
    : elevatorMG(elevatorPorts),
      judgeMG(judgePorts),
      liftMG(liftPorts),
      rollerMG(rollerPorts),
      cart(cartPistonsPort, Piston::RETRACTED),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort),
      acceptSensor(acceptSensorPort),
      rejectSensor(rejectSensorPort) {}

void Intake::configure(pros::MotorBrake brakeMode, pros::MotorGears gearset) {
  elevatorMG.set_brake_mode(brakeMode);
  elevatorMG.set_gearing(gearset);
  elevatorMG.set_encoder_units(pros::MotorEncoderUnits::degrees);
  elevatorMG.tare_position();

  judgeMG.set_brake_mode(brakeMode);
  judgeMG.set_gearing(gearset);
  judgeMG.set_encoder_units(pros::MotorEncoderUnits::degrees);
  judgeMG.tare_position();

  rollerMG.set_brake_mode(brakeMode);
  rollerMG.set_gearing(gearset);
  rollerMG.set_encoder_units(pros::MotorEncoderUnits::degrees);
  rollerMG.tare_position();

  // Power control keeps the lift's physical motor cartridge gearing.
  liftMG.set_brake_mode(pros::MotorBrake::hold);
}

void Intake::move(const int& rpm) {
  this->elevator(rpm);
  this->judge(rpm);
}

void Intake::elevator(const int& rpm) { elevatorMG.move_velocity(rpm); }

void Intake::judge(const int& rpm) { judgeMG.move_velocity(rpm); }

void Intake::roller(const int& rpm) { rollerMG.move_velocity(rpm); }

void Intake::scan() {
  size_t stopTime = UINT32_MAX;
  const short DELAY_PER_BALL = 2200;  // ms
  while (true) {
    if (scanning) {
      if (this->isObjectDetected()) {
        this->pickUp();
        stopTime = pros::millis() + DELAY_PER_BALL;
        pros::delay(500);  // this is to avoid counting the same block multiple times
      }
      // Only stop the elevator if sort isn't actively controlling it
      if (!releasing && pros::millis() >= stopTime) {
        this->elevator(0);
        stopTime = INT_MAX;
      }
    }
    pros::delay(50);
  }
}

void Intake::sort() {
  sortState = INIT;
  bool lastReleasing = false;
  bool armed = true;
  bool pendingCorrect = false;
  uint32_t timerEnd = 0;

  while (true) {
    const double hue = this->hue();
    const bool red = isRed(hue), blue = isBlue(hue);

    if (releasing) {
        if (!lastReleasing) {
          // Rising edge — start init reverse non-blocking
          this->score(Intake::BOTTOM);
          this->judge(-INTAKE_VELOCITY);
          timerEnd = pros::millis() + 350;
          sortState = INIT;
          armed = true;
          lastReleasing = true;
        }

        switch (sortState) {
        case INIT:
          // Wait for init reverse to finish, then go to IDLE
          if (pros::millis() >= timerEnd) {
            this->elevator(0);
            this->judge(0);
            sortState = IDLE;
          }
          break;

        case IDLE:
          this->elevator(INTAKE_VELOCITY * 2 / 3);
          if (armed && (red || blue)) {
            armed = false;
            pendingCorrect = (ALLIANCE == Alliance::Skills) ||
                             (red && ALLIANCE == Alliance::Red) || (blue && ALLIANCE == Alliance::Blue);
            const Height height = pendingCorrect ? acceptHeight : (acceptHeight == TOP ? MIDDLE : TOP);
            if (height != TOP) {
              // Kickback — reverse briefly before routing
              this->elevator(-INTAKE_VELOCITY);
              timerEnd = pros::millis() + 265;
              sortState = KICKBACK;
            } else {
              this->elevator(INTAKE_VELOCITY * 2 / 3);
              this->judge(INTAKE_VELOCITY);
              sortState = WAIT_ACCEPT;
            }
          }
          break;

        case KICKBACK:
          // Wait for kickback to finish, then spin judge
          if (pros::millis() >= timerEnd) {
            const Height height = pendingCorrect ? acceptHeight : (acceptHeight == TOP ? MIDDLE : TOP);
            this->elevator(INTAKE_VELOCITY * 2 / 3);
            this->judge(height == TOP ? INTAKE_VELOCITY : -INTAKE_VELOCITY);
            sortState = pendingCorrect ? WAIT_ACCEPT : WAIT_REJECT;
          }
          break;

        case WAIT_ACCEPT: {
          auto& sensor = (acceptHeight == TOP) ? acceptSensor : rejectSensor;
          if (sensor.isDetecting()) sortState = CONFIRM_ACCEPT;
          break;
        }

        case CONFIRM_ACCEPT: {
          auto& sensor = (acceptHeight == TOP) ? acceptSensor : rejectSensor;
          if (!sensor.isDetecting()) {
            this->judge(0);
            timerEnd = pros::millis() + 105;
            sortState = SETTLE;
          }
          break;
        }

        case WAIT_REJECT: {
          auto& sensor = (acceptHeight != TOP) ? acceptSensor : rejectSensor;
          if (sensor.isDetecting()) sortState = CONFIRM_REJECT;
          break;
        }

        case CONFIRM_REJECT: {
          auto& sensor = (acceptHeight != TOP) ? acceptSensor : rejectSensor;
          if (!sensor.isDetecting()) {
            this->judge(0);
            timerEnd = pros::millis() + 105;
            sortState = SETTLE;
          }
          break;
        }

        case SETTLE:
          // Brief pause after block clears before re-arming
          if (pros::millis() >= timerEnd) {
            sortState = IDLE;
            armed = true;
          }
          break;
        }
      } else {
        if (lastReleasing) {
          // Falling edge — stop motors and reset for next press
          this->elevator(0);
          this->judge(0);
          sortState = INIT;
          timerEnd = UINT32_MAX; // prevent stale timer from skipping init on next press
        }
        lastReleasing = false;
      }
    pros::delay(10);
  }
}

void Intake::pickUp(const int& delay) {
  this->elevator();
  if (delay == 0) return;
  pros::delay(delay);
  this->elevator(0);
}

void Intake::store(const int& delay) {
  this->elevator();
  if (delay == 0) return;
  pros::delay(delay);
  this->elevator(0);
}

void Intake::score(const Height& to, const int& delay) {
  if (to == TOP) {
    this->move();
  } else if (to == MIDDLE) {
    this->elevator();
    this->judge(-INTAKE_VELOCITY);
  } else if (to == BOTTOM) {
    this->elevator(-INTAKE_VELOCITY);
    this->judge(-INTAKE_VELOCITY);
  }
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void Intake::toggleCart() { cart.toggle(); }

void Intake::dropCart() { cart.activate(); }

void Intake::raiseCart() { cart.deactivate(); }

void Intake::activateScan() { scanning = true; }

void Intake::stopScan() { scanning = false; }

void Intake::setSortHeights(Height accept) { // TODO: rename
  acceptHeight = accept;
}

void Intake::startReleasing() {
  releasing = false;  // force falling edge so sort task fully resets
  pros::delay(10);    // give sort task one cycle to see the false
  releasing = true;
}

void Intake::stopReleasing() { releasing = false; }

Intake::SortState Intake::getSortingState() const { return sortState; }
  
#else

Intake::Intake(const std::initializer_list<std::int8_t>& intakePorts,
               const std::initializer_list<std::int8_t>& liftPorts,
               const std::initializer_list<std::int8_t>& clawPorts,
               const std::initializer_list<std::int8_t>& clawArmPorts,
               char scorerPistonPort, char cartPistonPort, char trapdoorPistonPort,
               int distanceSensorPort, int colorSensorPort)
    : intakeMG(intakePorts),
      liftMG(liftPorts),
      clawMG(clawPorts),
      clawArmMG(clawArmPorts),
      scorerPiston(scorerPistonPort, Piston::RETRACTED),
      cart(cartPistonPort, Piston::RETRACTED),
      trapdoor(trapdoorPistonPort, Piston::RETRACTED),
      distanceSensor(distanceSensorPort),
      colorSensor(colorSensorPort) {}

void Intake::configure(pros::MotorBrake brakeMode, pros::MotorGears gearset) {
  intakeMG.set_brake_mode(brakeMode);
  intakeMG.set_gearing(gearset);
  intakeMG.set_encoder_units(pros::MotorEncoderUnits::degrees);
  intakeMG.tare_position();

  // These mechanisms use power control and retain their motor cartridge gearing.
  liftMG.set_brake_mode(pros::MotorBrake::hold);
  clawMG.set_brake_mode(pros::MotorBrake::hold);
  clawArmMG.set_brake_mode(pros::MotorBrake::hold);
}

void Intake::move(const int& rpm) { this->corridor(rpm); }

void Intake::claw(const int& power) {
  if (power == 0) clawMG.brake();
  else clawMG.move(power);
}

void Intake::clawArm(const int& power) {
  if (power == 0) clawArmMG.brake();
  else clawArmMG.move(power);
}

void Intake::corridor(const int& rpm) { intakeMG.move_velocity(rpm); }

void Intake::elevator(const int& rpm) { liftMG.move_velocity(rpm); }

void Intake::judge(const int& rpm) { clawMG.move_velocity(rpm); }

void Intake::scan() {
  colorSensor.set_led_pwm(50);
  size_t stopTime = UINT32_MAX;
  const short DELAY_PER_BALL = 2450;  // ms
  while (true) {
    if (scanning) {
      if (this->isObjectDetected()) {
        this->pickUp();
        stopTime = pros::millis() + DELAY_PER_BALL;
        pros::delay(
            500);  // this is to avoid counting the same block many times
      }

      if (pros::millis() >= stopTime) {
        this->corridor(0);
        stopTime = UINT32_MAX;
      }
    }

    pros::delay(50);
  }
}

void Intake::sort() {
  while (true) {
    if (scanning) {
      const double hue = this->hue();
      const bool red = isRed(hue), blue = isBlue(hue);

      if (red || blue) {
        if (ALLIANCE != Alliance::Skills && ((red && ALLIANCE == Alliance::Blue) || (blue && ALLIANCE == Alliance::Red))) {
          // Wrong alliance color detected — reverse motor to eject
          this->corridor(-INTAKE_VELOCITY);
          pros::delay(100);
          this->corridor(0);
        } else if ((red && ALLIANCE == Alliance::Red) || (blue && ALLIANCE == Alliance::Blue)) {
        // Correct alliance color detected — move motor to accept
          this->corridor(INTAKE_VELOCITY);
          pros::delay(125);
          this->corridor(0);
        }
      }
    }
    pros::delay(25);
  }
}

void Intake::pickUp(const int& delay) {
  this->corridor();
  if (delay == 0) return;
  pros::delay(delay);
  this->corridor(0);
}

void Intake::store(const int& delay) {
  this->move();
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void Intake::reject(const int& delay) {
  this->corridor(-INTAKE_VELOCITY);
  if (delay == 0) return;
  pros::delay(delay);
  this->corridor(0);
}

void Intake::lever(const uint32_t timeout) {
  this->extendLever();
  Timer timer(timeout);
  while(!this->leverFinished() && !timer.isCompleted()){ pros::delay(5); }
  this->resetLever();
}

void Intake::extendLever() {
  leverTarget = 150;
  clawArmMG.move_absolute(leverTarget, 100);
}

void Intake::resetLever() {
  leverTarget = 0;
  clawArmMG.move_absolute(leverTarget, 100);
}

bool Intake::leverFinished() {
  return std::abs(clawArmMG.get_position() - leverTarget) < 10;
}

void Intake::score(const Height& height, const int& delay) {
  if (height == TOP) {
    this->store();
  } else if (height == BOTTOM) {
    this->move(-INTAKE_VELOCITY);
  } else
    return;
  if (delay == 0) return;
  pros::delay(delay);
  this->stop();
}

void Intake::toggleScorerHeight() { scorerPiston.toggle(); }

void Intake::raiseScorer() { scorerPiston.activate(); }

void Intake::lowerScorer() { scorerPiston.deactivate(); }

void Intake::toggleCart() { cart.toggle(); }

void Intake::dropCart() { cart.activate(); }

void Intake::raiseCart() { cart.deactivate(); }

void Intake::toggleTrapdoor() { trapdoor.toggle(); }

void Intake::openTrapdoor() { trapdoor.activate(); }

void Intake::closeTrapdoor() { trapdoor.deactivate(); }

void Intake::scorer(const int& rpm) {
  clawArmMG.move_velocity(rpm);
}

void Intake::activateScan() {
  scanning = true;
}

void Intake::stopScan() {
  scanning = false;
  this->corridor(0);
}

#endif

void Intake::lift(const int& power) {
  if (power == 0) liftMG.brake();
  else liftMG.move(power);
}

void Intake::stop() {
  this->move(0);
  this->lift(0);
#if USING_BIG_ROBOT
  this->roller(0);
#else
  this->claw(0);
  this->clawArm(0);
#endif
}

double Intake::distance() { return distanceSensor.get(); }

bool Intake::isObjectDetected() { return this->distance() <= INTAKE_ACTIVATION_DISTANCE; }

bool Intake::isScanning() { return this->scanning; }

void Intake::kickBack() {
  this->move(-100);
  pros::delay(150);
  this->stop();
}

double Intake::hue() { return colorSensor.get_hue(); }

void Intake::setScoreDown(bool down) { scoreDown = down; }

bool Intake::isRed(const double& hue) { return (hue >= 356 && hue <= 359) || (hue >= 1 && hue <= 25); }
bool Intake::isBlue(const double& hue) { return 170 <= hue && hue <= 230; }

}  // namespace aon
