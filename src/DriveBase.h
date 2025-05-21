#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "main.h"
#include "RobotValues.h"
#include <initializer_list>
#include <vector>

class DriveBase
{
public:
  /**
   * @param controller  tje pros::Controller reference
   * @param leftPorts   list of left-side ports; use a negative number to mark reversed
   * @param rightPorts  same
   * @param gear        which gearset to use (defaults to RobotValues::DRIVE_GEAR)
   */
  DriveBase(pros::Controller &controller,
            std::initializer_list<int> leftPorts,
            std::initializer_list<int> rightPorts,
            pros::motor_gearset_e gear = RobotValues::DRIVE_GEAR)
      : _ctrl(controller)
  {
    for (auto port : leftPorts)
    {
      bool rev = port < 0;
      uint8_t p = static_cast<uint8_t>(rev ? -port : port);
      _leftMotors.emplace_back(p, gear, rev);
    }
    for (auto port : rightPorts)
    {
      bool rev = port < 0;
      uint8_t p = static_cast<uint8_t>(rev ? -port : port);
      _rightMotors.emplace_back(p, gear, rev);
    }
  }

  // calls in every cycle in opcontrol()
  void arcadeDrive()
  {
    int pwr = _ctrl.get_analog(ANALOG_LEFT_Y);
    int turn = _ctrl.get_analog(ANALOG_RIGHT_X);
    int leftV = pwr + turn;
    int rightV = pwr - turn;

    for (auto &m : _leftMotors)
      m.move(leftV);
    for (auto &m : _rightMotors)
      m.move(rightV);
  }

private:
  pros::Controller &_ctrl;
  std::vector<pros::Motor> _leftMotors;
  std::vector<pros::Motor> _rightMotors;
};

#endif