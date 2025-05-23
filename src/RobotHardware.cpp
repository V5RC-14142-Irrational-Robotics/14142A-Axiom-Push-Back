#include "RobotHardware.h"
#include <tuple>
#include <cmath>

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
    case State::IDLE:       driveBase.stop();        break;
    case State::DRIVER:     driveBase.arcadeDrive(); break;
    case State::AUTONOMOUS:  break;
  }

  localizer.motionUpdate();
  localizer.measurementUpdate();
  localizer.resample();
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

bool RobotHardware::moveToXYH(double tx,
                              double ty,
                              double targetH,
                              double fwdPower,
                              double distTol,
                              double headingTol,
                              double headingGain) {
  // current esitmate
  auto [x, y, theta] = localizer.getEstimate();
  double dx   = tx - x;
  double dy   = ty - y;
  double dist = std::hypot(dx, dy);

  // phase 1 drive toward the waypoint
  if (dist > distTol) {
    double absAng = std::atan2(dy, dx) * 180.0 / M_PI;
    driveBase.driveYH(fwdPower, absAng, headingGain);
    return false;
  }

  // phase 2: rotate to final heading
  double errH = std::fabs(
    std::fmod((targetH - theta + 540.0), 360.0) - 180.0
  ); 
  if (errH > headingTol) {
    driveBase.driveYH(0.0, targetH, headingGain);
    return false;
  }

  driveBase.stop();
  return true;
}

void RobotHardware::setState(State s) {
  fsm.set(s);
  update();
}

RobotHardware::State RobotHardware::getState() const {
  return fsm.get();
}
