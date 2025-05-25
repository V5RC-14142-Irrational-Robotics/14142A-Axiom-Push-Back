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

  /**
   * this moves to the target field (x,y) in inches, then face targetH (deg).
   * it returns true once within distTol and headingTol (cus we can tbe exact).
   * fwdPower in [-127..127], distTol & headingTol in units, headingGain in deg -> turn gain.
   */
  bool moveToXYH(double tx,
                 double ty,
                 double targetH,
                 double fwdPower    = 127.0,
                 double distTol     = 1.0,
                 double headingTol  = 3.0,
                 double headingGain = 0.006);

private:
  Telemetry           telem;
  StateMachine<State> fsm;

  void updateDashboard();
};

#endif
