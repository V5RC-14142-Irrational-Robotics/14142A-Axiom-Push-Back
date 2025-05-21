#include "DriveBase.h"
#include <cmath>

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
  int pwr  = _master.get_analog(ANALOG_LEFT_Y);
  int turn = _master.get_analog(ANALOG_RIGHT_X);
  int L = pwr + turn;
  int R = pwr - turn;
  for (auto &m : _leftMotors)  m.move(L);
  for (auto &m : _rightMotors) m.move(R);
}

void DriveBase::recordCurrents() {
  auto log = [&](pros::Motor &m, std::vector<double> &hist){
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

double DriveBase::getHeading() {
  return _imu.get_heading();
}

bool DriveBase::isHeading(double targetH, double tolDeg) {
  double err = normalizeAngle(targetH - getHeading());
  return std::fabs(err) < tolDeg;
}

double DriveBase::calcRVW(double targetH, double gain) {
  double err = normalizeAngle(targetH - getHeading());
  double w   = err * gain;
  return std::clamp(w, -1.0, 1.0);
}

void DriveBase::driveXYW(double rx, double ry, double rw, double vel) {
  double D  = std::max(std::fabs(ry) + std::fabs(rx) + std::fabs(rw), 1.0);
  double lf = ( rx - ry - rw) / D;
  double rf = ( rx + ry + rw) / D;
  double lb = ( rx + ry - rw) / D;
  double rb = ( rx - ry + rw) / D;

  int L = int(lf * vel * 127.0);
  int R = int(rf * vel * 127.0);
  for (auto &m : _leftMotors)  m.move(L);
  for (auto &m : _rightMotors) m.move(R);
}

bool DriveBase::driveXYH(double rx, double ry, double targetH, double gain) {
  double w = calcRVW(targetH, gain);
  driveXYW(rx, ry, w, 1.0);
  return false;
}

void DriveBase::driveFieldXYW(double fx, double fy, double rw, double vel) {
  double theta  = getHeading() * M_PI / 180.0;
  double rx = fx * std::cos(-theta) - fy * std::sin(-theta);
  double ry = fx * std::sin(-theta) + fy * std::cos(-theta);
  driveXYW(rx, ry, rw, vel);
}

bool DriveBase::stop() {
  for (auto &m : _leftMotors)  m.move(0);
  for (auto &m : _rightMotors) m.move(0);
  return true;
}

double DriveBase::normalizeAngle(double deg) {
  while (deg >  180.0) deg -= 360.0;
  while (deg < -180.0) deg += 360.0;
  return deg;
}
