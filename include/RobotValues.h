#ifndef ROBOTVALUES_H
#define ROBOTVALUES_H

#include "main.h"
#include <initializer_list>

namespace RobotValues {
  // + = forward, - = reversed
  inline const std::initializer_list<int> LEFT_DRIVE_PORTS  {  1, -2,  3 };
  inline const std::initializer_list<int> RIGHT_DRIVE_PORTS { -4,  5, -6 };

  constexpr pros::v5::MotorGears DRIVE_GEAR   = pros::v5::MotorGears::blue;
  constexpr pros::v5::MotorUnits ENCODER_UNITS = pros::v5::MotorUnits::degrees;

  constexpr int IMU_PORT = 7;

  constexpr double MAX_VELOCITY = 200.0;
}

#endif