#ifndef ROBOTHARDWARE_H
#define ROBOTHARDWARE_H

#include "main.h"
#include "StateMachine.h"
#include "DriveBase.h"

class RobotHardware {
public:
  enum class State {
    IDLE,
    DRIVER,
    AUTONOMOUS
  };

  explicit RobotHardware(pros::Controller &master)
    : driveBase(master), fsm(State::IDLE) {}

  void init() {
    driveBase.init();
  }

  void update() {
    switch (fsm.get()) {
      case State::IDLE:
        driveBase.stop();
        break;
      case State::DRIVER:
        driveBase.arcadeDrive();
        break;
      case State::AUTONOMOUS:
        //auton logic in main.cpp
        driveBase.stop();
        break;
    }
  }

  void setState(State s) { fsm.set(s); }
  State getState() const { return fsm.get(); }

  DriveBase driveBase;

private:
  StateMachine<State> fsm;
};

#endif
