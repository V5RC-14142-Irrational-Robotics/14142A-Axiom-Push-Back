#ifndef ROBOTVALUES_H
#define ROBOTVALUES_H

#include "main.h"
#include <initializer_list>

namespace RobotValues {
  inline const std::initializer_list<int> LEFT_DRIVE_PORTS  {  1, -2,  3 };
  inline const std::initializer_list<int> RIGHT_DRIVE_PORTS { -4,  5, -6 };

  // fix: must be the PROS enum, not a pros::v5::MotorGears type
  constexpr pros::motor_gearset_e DRIVE_GEAR = pros::E_MOTOR_GEARSET_18;

  constexpr int IMU_PORT = 7;
}

#endif
