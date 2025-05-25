#include "RobotHardware.h"
#include <tuple>
#include <cmath>

RobotHardware::RobotHardware(pros::Controller &master)
  : driveBase(master),
    fsm(State::IDLE),
    telem()
{}

void RobotHardware::init() {
  driveBase.init();
  telem.clear();
  telem.addLine("Status", "Initialized");
  telem.display();
}

void RobotHardware::update() {
  switch (fsm.get()) {
    case State::IDLE:       driveBase.stop();        break;
    case State::DRIVER:     driveBase.arcadeDrive(); break;
    case State::AUTONOMOUS:  break;
  }

  updateDashboard();
  telem.display();
}

void RobotHardware::updateDashboard() {
  telem.clear();
  switch (fsm.get()) {
    case State::IDLE:       telem.addLine("Mode", "IDLE");    break;
    case State::DRIVER:     telem.addLine("Mode", "DRIVER");  break;
    case State::AUTONOMOUS: telem.addLine("Mode", "AUTONOM"); break;
  }

}

void RobotHardware::setState(State s) {
  fsm.set(s);
  update();
}

RobotHardware::State RobotHardware::getState() const {
  return fsm.get();
}
