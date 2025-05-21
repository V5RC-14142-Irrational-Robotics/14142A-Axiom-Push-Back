#ifndef ROBOTHARDWARE_H
#define ROBOTHARDWARE_H

#include "main.h"
#include "RobotValues.h"
#include "StateMachine.h"
#include "DriveBase.h"

class RobotHardware {
public:
  enum class State {
    IDLE,
    //we cab add more states here
  };

  RobotHardware(pros::Controller &master)
    : ctrl(master),
      imu(RobotValues::IMU_PORT),
      driveBase(ctrl,
                RobotValues::LEFT_DRIVE_PORTS,
                RobotValues::RIGHT_DRIVE_PORTS,
                RobotValues::DRIVE_GEAR),
      fsm(State::IDLE)
  {}


  void init() {
    imu.reset();
    //calibrate() we need to create this function
  }

  /// calls in every cycle in opcontrol()
  void update() {
    switch (fsm.get()) {
      case State::IDLE:
        driveBase.arcadeDrive();
        break;
    }
  }

  void setState(State s) { fsm.set(s); }

  State getState() const { return fsm.get(); }

private:
  pros::Controller    &ctrl;
  pros::Imu             imu;
  DriveBase            driveBase;
  StateMachine<State>   fsm;

  //we can add stuff like this
  // void handleIntake() {insert stuff}
};

#endif
