#include "RobotHardware.h"

RobotHardware::RobotHardware(pros::Controller &master)
  : driveBase(master),
    fsm(State::IDLE)
{}

void RobotHardware::init() {
  driveBase.init();
}

void RobotHardware::update() {
  switch (fsm.get()) {
    case State::IDLE:
      driveBase.stop();
      break;

    case State::DRIVER:
      driveBase.arcadeDrive();
      break;

    case State::AUTONOMOUS:
      driveBase.stop();
      break;
  }
}

void RobotHardware::setState(State s) {
  fsm.set(s);
}

RobotHardware::State RobotHardware::getState() const {
  return fsm.get();
}
