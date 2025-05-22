#include "RobotHardware.h"

RobotHardware::RobotHardware(pros::Controller &master)
  : driveBase(master),
    fsm(State::IDLE),
    telem()
{}

void RobotHardware::init() {
  driveBase.init();
  updateDashboard();
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
      break;
  }
  updateDashboard();
}

void RobotHardware::setState(State s) {
  fsm.set(s);
  updateDashboard();
}

RobotHardware::State RobotHardware::getState() const {
  return fsm.get();
}

void RobotHardware::updateDashboard() {
  telem.clear();

  switch (fsm.get()) {
    case State::IDLE:       telem.addLine("Mode", "IDLE");       break;
    case State::DRIVER:     telem.addLine("Mode", "DRIVER");     break;
    case State::AUTONOMOUS: telem.addLine("Mode", "AUTON");      break;
  }

  telem.addLine("Heading", [this](){
    return std::to_string(driveBase.imuGetHeading());
  });

  telem.addLine("LF Pos", [this](){
    return std::to_string(driveBase.getLeftPosition());
  });
  telem.addLine("RF Pos", [this](){
    return std::to_string(driveBase.getRightPosition());
  });

  telem.display();
}
