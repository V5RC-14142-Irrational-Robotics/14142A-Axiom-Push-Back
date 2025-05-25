#ifndef DRIVEBASE_H
#define DRIVEBASE_H

#include "main.h"
#include "RobotValues.h"
#include "pros/imu.hpp"
#include <vector>

class DriveBase {
public:
  explicit DriveBase(pros::Controller &master);

  void init();
  void arcadeDrive();

  std::vector<double> getCurrents();

  // int getLeftPosition() const;
  // int getRightPosition() const;
  
  // Robotcentric
  void driveYW(double forward, double turn, double vel = 1.0);
  bool driveYH(double forward, double targetH, double gain = 0.006);

  // Fieldcentric
  void driveFieldYW(double fw, double turn, double vel = 1.0);
  void driveDistance(double inches, double vel = 1.0);

  // IMU accessors
  std::int32_t    imuReset(bool blocking = false);
  double          imuGetRotation();
  double          imuGetHeading();
  pros::quaternion_s_t imuGetQuaternion();
  pros::euler_s_t      imuGetEuler();
  double          imuGetPitch();
  double          imuGetRoll();
  double          imuGetYaw();
  pros::imu_gyro_s_t   imuGetGyroRate();
  pros::imu_accel_s_t  imuGetAccel();
  bool            imuIsCalibrating();

  // Tare (zero) ops
  std::int32_t imuTareRotation();
  std::int32_t imuTareHeading();
  std::int32_t imuTarePitch();
  std::int32_t imuTareRoll();
  std::int32_t imuTareYaw();
  std::int32_t imuTare();
  std::int32_t imuTareEuler();

  // Manual set orientation
  std::int32_t imuSetHeading(double heading);
  std::int32_t imuSetRotation(double rotation);
  std::int32_t imuSetYaw(double yaw);
  std::int32_t imuSetPitch(double pitch);
  std::int32_t imuSetRoll(double roll);
  std::int32_t imuSetEuler(const pros::euler_s_t &euler);
  pros::Imu &getImu() { return _imu; }

  int getLeftPosition() const;
  int getRightPosition() const;

  // Emergency stop
  bool stop();

private:
  pros::Controller &_master;
  pros::Imu          _imu;
  std::vector<pros::Motor> _leftMotors, _rightMotors;
  std::vector<double>      _lfHist, _lbHist, _rfHist, _rbHist;


  double            calcRVW(double targetH, double gain);
  static double     normalizeAngle(double deg);

  static double inchesToDegrees(double inches) {
    return (inches / RobotValues::WHEEL_CIRCUMFERENCE) * 360.0;
  }
};

#endif
