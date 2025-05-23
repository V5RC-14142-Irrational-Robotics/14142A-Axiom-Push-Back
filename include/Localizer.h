#ifndef LOCALIZER_H
#define LOCALIZER_H

#include <vector>
#include <random>
#include <tuple>
#include "RobotValues.h"
#include "DriveBase.h"
#include "pros/imu.hpp"
#include "pros/distance.hpp"

struct Particle {
  double x, y, theta;   // pose in inches & degrees
  double weight;
};

/// Particle‐filter localizer using diff‐drive odometry, IMU, & 4 range sensors
class Localizer {
public:
  Localizer(DriveBase             &drive,
            pros::Imu             &imu,
            pros::Distance        &frontDist,
            pros::Distance        &rightDist,
            pros::Distance        &backDist,
            pros::Distance        &leftDist);

  /// Uniformly scatter particles across the field
  void initUniform();

  /// 1) Propagate via motion model
  void motionUpdate();

  /// 2) Weight via range‐measurement model
  void measurementUpdate();

  /// 3) Resample particles
  void resample();

  /// Get the current pose estimate (x, y, theta)
  std::tuple<double,double,double> getEstimate() const;

private:
  std::vector<Particle> particles;
  DriveBase    &drive;
  pros::Imu    &imu;
  pros::Distance &front, &right, &back, &left;
  std::mt19937 rng;
  double prevLeftDeg  = 0.0;
  double prevRightDeg = 0.0;
  
  // Helpers
  double inchesPerDegree() const {
    return RobotValues::WHEEL_DIAMETER_INCH * M_PI / 360.0;
  }
  double gaussian(double mu, double sigma, double x);
};

#endif // LOCALIZER_H
