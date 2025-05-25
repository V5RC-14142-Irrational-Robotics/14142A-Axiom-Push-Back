#ifndef ROBOTVALUES_H
#define ROBOTVALUES_H

#include "main.h"
#include <initializer_list>

namespace RobotValues {
  // + = forward, - = reversed
  inline const std::initializer_list<int> LEFT_DRIVE_PORTS  {  -11, 12,  -13 };
  inline const std::initializer_list<int> RIGHT_DRIVE_PORTS { 18,  -19, 20 };

  constexpr int DIST_FRONT_PORT  = 1; //change ltr
  constexpr int DIST_RIGHT_PORT  = 2;
  constexpr int DIST_BACK_PORT   = 3;
  constexpr int DIST_LEFT_PORT   = 4;

  constexpr int IMU_PORT = 7;

  constexpr pros::v5::MotorGears DRIVE_GEAR   = pros::v5::MotorGears::blue;
  constexpr pros::v5::MotorUnits ENCODER_UNITS = pros::v5::MotorUnits::degrees;

  constexpr double MAX_VELOCITY = 200.0;

  constexpr double WHEEL_DIAMETER_INCH = 2.75;
  constexpr double WHEEL_CIRCUMFERENCE = WHEEL_DIAMETER_INCH * M_PI;

  constexpr double FIELD_WIDTH_IN  = 144.0;
  constexpr double FIELD_HEIGHT_IN = 144.0;

  constexpr double WHEEL_BASE_IN   = 9.558;

}

#endif