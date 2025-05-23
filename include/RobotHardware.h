#ifndef ROBOTHARDWARE_H
#define ROBOTHARDWARE_H

#include "main.h"
#include "RobotValues.h"
#include "DriveBase.h"
#include "StateMachine.h"
#include "Telemetry.h"
#include "Localizer.h"
#include <pros/distance.hpp>

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
  Telemetry          telem;
  pros::Distance     frontDist, rightDist, backDist, leftDist;
  Localizer          localizer;
  StateMachine<State> fsm;

  void updateDashboard();
};

#endif