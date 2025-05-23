#include "RobotHardware.h"
#include <tuple>
RobotHardware::RobotHardware(pros::Controller &master)
  : driveBase(master),
    frontDist(RobotValues::DIST_FRONT_PORT),
    rightDist(RobotValues::DIST_RIGHT_PORT),
    backDist(RobotValues::DIST_BACK_PORT),
    leftDist(RobotValues::DIST_LEFT_PORT),
    localizer(driveBase, driveBase.getImu(),
              frontDist, rightDist, backDist, leftDist),
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
    case State::IDLE:       driveBase.stop();         break;
    case State::DRIVER:     driveBase.arcadeDrive();  break;
    case State::AUTONOMOUS: break;
  }

  localizer.motionUpdate();
  localizer.measurementUpdate();
  localizer.resample();
  auto [x, y, theta] = localizer.getEstimate();

  updateDashboard();

  telem.addLine("X",   [x]()     { return std::to_string(x);     });
  telem.addLine("Y",   [y]()     { return std::to_string(y);     });
  telem.addLine("Th",  [theta](){ return std::to_string(theta); });
  telem.addLine("HDG", [this](){
    return std::to_string(driveBase.imuGetHeading());
  });
  telem.addLine("FDst", [this](){
    return std::to_string(frontDist.get());
  });
  telem.addLine("LPos", [this](){
    return std::to_string(driveBase.getLeftPosition());
  });
  telem.addLine("RPos", [this](){
    return std::to_string(driveBase.getRightPosition());
  });

  telem.display();
}

void RobotHardware::updateDashboard() {
  telem.clear();
  switch (fsm.get()) {
    case State::IDLE:       telem.addLine("Mode", "IDLE");       break;
    case State::DRIVER:     telem.addLine("Mode", "DRIVER");     break;
    case State::AUTONOMOUS: telem.addLine("Mode", "AUTONOM");    break;
  }
}

void RobotHardware::setState(State s) {
  fsm.set(s);
  update();
}

RobotHardware::State RobotHardware::getState() const {
  return fsm.get();
}
