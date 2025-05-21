#ifndef ROBOTHARDWARE_H
#define ROBOTHARDWARE_H

#include "DriveBase.h"
#include "StateMachine.h"

class RobotHardware {
public:
  enum class State {
    IDLE,
    DRIVER,
    AUTONOMOUS
  };

  explicit RobotHardware(pros::Controller &master);
  void init();
  void update();

  void setState(State s);
  State getState() const;

  DriveBase driveBase;

private:
  StateMachine<State> fsm;
};

#endif