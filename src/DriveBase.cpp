#include "DriveBase.h"
#include <cmath>
#include <algorithm>

DriveBase::DriveBase(pros::Controller &master)
  : _master(master),
    _imu(RobotValues::IMU_PORT)
{
  for (int port : RobotValues::LEFT_DRIVE_PORTS) {
    pros::Motor m{port, RobotValues::DRIVE_GEAR, RobotValues::ENCODER_UNITS};
    m.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    _leftMotors.push_back(std::move(m));
  }
  for (int port : RobotValues::RIGHT_DRIVE_PORTS) {
    pros::Motor m{port, RobotValues::DRIVE_GEAR, RobotValues::ENCODER_UNITS};
    m.set_brake_mode(pros::E_MOTOR_BRAKE_BRAKE);
    _rightMotors.push_back(std::move(m));
  }
}

void DriveBase::init() {
  _imu.reset(true);
  pros::delay(100);
}

void DriveBase::arcadeDrive() {
  int p = _master.get_analog(ANALOG_LEFT_Y);
  int t = _master.get_analog(ANALOG_RIGHT_X);
  int L = p + t, R = p - t;
  for (auto &m : _leftMotors)  m.move(L);
  for (auto &m : _rightMotors) m.move(R);
}

void DriveBase::recordCurrents() {
  auto log = [&](pros::Motor &m, auto &hist){
    double c = m.get_current_draw();
    if (c > 3.5) hist.push_back(c);
  };
  for (auto &m : _leftMotors)  log(m, _lfHist);
  for (auto &m : _leftMotors)  log(m, _lbHist);
  for (auto &m : _rightMotors) log(m, _rfHist);
  for (auto &m : _rightMotors) log(m, _rbHist);
}

void DriveBase::displayHistory() {
  std::string out;
  for (double v : _lfHist) out += std::to_string(v) + " ";
  pros::lcd::clear();
  pros::lcd::set_text(1, out.c_str());
}

// drive methods
void DriveBase::driveYW(double forward, double turn, double vel) {
  double sum = std::fabs(forward) + std::fabs(turn);
  double D   = std::max(sum, 1.0);
  double lf  = ( forward - turn) / D;
  double rf  = ( forward + turn) / D;

  int L = int(lf * vel * 127.0);
  int R = int(rf * vel * 127.0);

  for (auto &m : _leftMotors)  m.move(L);
  for (auto &m : _rightMotors) m.move(R);
}

bool DriveBase::driveYH(double forward, double targetH, double gain) {
  double w = calcRVW(targetH, gain);
  driveYW(forward, w * 127.0, 1.0);
  return false;
}

void DriveBase::driveFieldYW(double fw, double turn, double vel) {
  double theta = imuGetHeading() * M_PI/180.0;
  double robotForward = fw * std::cos(-theta);
  driveYW(robotForward * 127.0, turn, vel);
}

void DriveBase::driveDistance(double inches, double vel) {
  double targetDeg = inchesToDegrees(inches);

  for (auto &m : _leftMotors)  m.tare_position();
  for (auto &m : _rightMotors) m.tare_position();

  for (auto &m : _leftMotors)  m.move_relative(targetDeg, vel * 127.0);
  for (auto &m : _rightMotors) m.move_relative(targetDeg, vel * 127.0);

  while (true) {
    if (std::abs(_leftMotors.front().get_position()) >= targetDeg) break;
    pros::delay(10);
  }
}

bool DriveBase::stop() {
  for (auto &m : _leftMotors)  m.move(0);
  for (auto &m : _rightMotors) m.move(0);
  return true;
}

//imu stuff

std::int32_t DriveBase::imuReset(bool blocking)               { return _imu.reset(blocking); }
double       DriveBase::imuGetRotation()                      { return _imu.get_rotation(); }
double       DriveBase::imuGetHeading()                       { return _imu.get_heading(); }
pros::quaternion_s_t DriveBase::imuGetQuaternion()            { return _imu.get_quaternion(); }
pros::euler_s_t      DriveBase::imuGetEuler()                 { return _imu.get_euler(); }
double       DriveBase::imuGetPitch()                         { return _imu.get_pitch(); }
double       DriveBase::imuGetRoll()                          { return _imu.get_roll(); }
double       DriveBase::imuGetYaw()                           { return _imu.get_yaw(); }
pros::imu_gyro_s_t   DriveBase::imuGetGyroRate()              { return _imu.get_gyro_rate(); }
pros::imu_accel_s_t  DriveBase::imuGetAccel()                 { return _imu.get_accel(); }
bool         DriveBase::imuIsCalibrating()                    { return _imu.is_calibrating(); }

std::int32_t DriveBase::imuTareRotation()                     { return _imu.tare_rotation(); }
std::int32_t DriveBase::imuTareHeading()                      { return _imu.tare_heading(); }
std::int32_t DriveBase::imuTarePitch()                        { return _imu.tare_pitch(); }
std::int32_t DriveBase::imuTareRoll()                         { return _imu.tare_roll(); }
std::int32_t DriveBase::imuTareYaw()                          { return _imu.tare_yaw(); }
std::int32_t DriveBase::imuTare()                             { return _imu.tare(); }
std::int32_t DriveBase::imuTareEuler()                        { return _imu.tare_euler(); }

std::int32_t DriveBase::imuSetHeading(double h)               { return _imu.set_heading(h); }
std::int32_t DriveBase::imuSetRotation(double r)              { return _imu.set_rotation(r); }
std::int32_t DriveBase::imuSetYaw(double y)                   { return _imu.set_yaw(y); }
std::int32_t DriveBase::imuSetPitch(double p)                 { return _imu.set_pitch(p); }
std::int32_t DriveBase::imuSetRoll(double r)                  { return _imu.set_roll(r); }
std::int32_t DriveBase::imuSetEuler(const pros::euler_s_t &e) { return _imu.set_euler(e); }

double DriveBase::calcRVW(double targetH, double gain) {
  double err = normalizeAngle(targetH - imuGetHeading());
  double w   = err * gain;
  return std::clamp(w, -1.0, 1.0);
}

double DriveBase::normalizeAngle(double deg) {
  while (deg >  180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}
