#include "Localizer.h"
#include <algorithm>
#include <numeric>
#include <cmath>

Localizer::Localizer(DriveBase &drv, pros::Imu &im,
                     pros::Distance &f, pros::Distance &r,
                     pros::Distance &b, pros::Distance &l)
  : drive(drv), imu(im),
    front(f), right(r), back(b), left(l),
    rng(std::random_device{}())
{
  particles.resize(RobotValues::NUM_PARTICLES);
  initUniform();
  prevLeftDeg  = drive.getLeftPosition();
  prevRightDeg = drive.getRightPosition();
}

void Localizer::initUniform() {
  std::uniform_real_distribution<double> ux(0, RobotValues::FIELD_WIDTH_IN);
  std::uniform_real_distribution<double> uy(0, RobotValues::FIELD_HEIGHT_IN);
  std::uniform_real_distribution<double> uth(0, 360);
  for (auto &p : particles) {
    p.x = ux(rng);
    p.y = uy(rng);
    p.theta = uth(rng);
    p.weight = 1.0 / particles.size();
  }
}

void Localizer::motionUpdate() {
  double curL = drive.getLeftPosition();
  double curR = drive.getRightPosition();
  double dL = (curL  - prevLeftDeg)  * inchesPerDegree();
  double dR = (curR  - prevRightDeg) * inchesPerDegree();
  prevLeftDeg  = curL;
  prevRightDeg = curR;

  double forward = (dL + dR) / 2.0;
  double dtheta  = imu.get_rotation() - 0.0;

  std::normal_distribution<double> nf(0, RobotValues::MOTION_STD_FWD);
  std::normal_distribution<double> na(0, RobotValues::MOTION_STD_ANG);

  for (auto &p : particles) {
    double noisyF = forward + nf(rng);
    double noisyA = dtheta + na(rng);
    p.theta   += noisyA;
    p.x       += noisyF * std::cos(p.theta * M_PI/180.0);
    p.y       += noisyF * std::sin(p.theta * M_PI/180.0);
  }
}

void Localizer::measurementUpdate() {
  double zF = front.get();
  double zR = right.get();
  double zB = back.get();
  double zL = left.get();

  for (auto &p : particles) {
    double eF = RobotValues::FIELD_HEIGHT_IN - p.y;
    double eB = p.y;
    double eR = RobotValues::FIELD_WIDTH_IN  - p.x;
    double eL = p.x;

    double w  = gaussian(eF, RobotValues::SENSE_STD, zF)
              * gaussian(eB, RobotValues::SENSE_STD, zB)
              * gaussian(eR, RobotValues::SENSE_STD, zR)
              * gaussian(eL, RobotValues::SENSE_STD, zL);
    p.weight = w + 1e-9;
  }
  double sumW = 0;
  for (auto &p : particles) sumW += p.weight;
  for (auto &p : particles) p.weight /= sumW;
}

void Localizer::resample() {
  std::vector<Particle> newP;
  newP.reserve(particles.size());
  std::vector<double> cdf(particles.size());
  cdf[0] = particles[0].weight;
  for (size_t i = 1; i < particles.size(); i++)
    cdf[i] = cdf[i-1] + particles[i].weight;

  std::uniform_real_distribution<double> runif(0,1.0);
  for (size_t i = 0; i < particles.size(); i++) {
    double r = runif(rng);
    auto it = std::lower_bound(cdf.begin(), cdf.end(), r);
    size_t idx = std::distance(cdf.begin(), it);
    newP.push_back(particles[idx]);
    newP.back().weight = 1.0/particles.size();
  }
  particles.swap(newP);
}

std::tuple<double,double,double> Localizer::getEstimate() const {
  double x=0,y=0,th=0;
  for (auto &p : particles) {
    x  += p.x * p.weight;
    y  += p.y * p.weight;
    th += p.theta * p.weight;
  }
  return {x,y,th};
}

double Localizer::gaussian(double mu, double sigma, double x) {
  double d = x - mu;
  return std::exp(-0.5 * (d*d)/(sigma*sigma)) / (sigma * std::sqrt(2*M_PI));
}
