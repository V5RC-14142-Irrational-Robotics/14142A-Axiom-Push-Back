#ifndef ROBOTHARDWARE_H
#define ROBOTHARDWARE_H

#include "main.h"
#include "RobotValues.h"
#include "DriveBase.h"
#include "StateMachine.h"
#include "Telemetry.h"

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
  Telemetry           telem;
  StateMachine<State> fsm;

  void updateDashboard();
};

#endif
