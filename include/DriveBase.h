#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "main.h"
#include "RobotValues.h"
#include <vector>

class DriveBase {
public:
  explicit DriveBase(pros::Controller &master);

  void init();
  void arcadeDrive();

  void recordCurrents();
  void displayHistory();

  /// Drive helpers
  void driveXYW(double rx, double ry, double rw, double vel = 1.0);
  bool driveXYH(double rx, double ry, double targetH, double gain = 0.006);
  void driveFieldXYW(double fx, double fy, double rw, double vel = 1.0);

  /// IMU helpers
  double getHeading();
  bool   isHeading(double targetH, double tolDeg);
  double calcRVW(double targetH, double gain = 0.006);

  /// Emergency (zero all motors)
  bool stop();

private:
  pros::Controller &_master;
  pros::Imu          _imu;
  std::vector<pros::Motor> _leftMotors, _rightMotors;
  std::vector<double>      _lfHist, _lbHist, _rfHist, _rbHist;

  static double normalizeAngle(double deg);
};

#endif
