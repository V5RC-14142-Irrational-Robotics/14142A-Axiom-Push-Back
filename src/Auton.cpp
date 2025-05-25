#include "Auton.h"

AutonMode selectedAuton = AutonMode::NONE;

void runSkills(DriveBase &drive) {
  drive.driveDistance(24*4, 1); // Example: Drive forward 24 inches at 100% speed
  drive.stop();
  pros::delay(500);
  
  // Add more skills tasks here
  // e.g., drive.turn(90.0); or drive.driveDistance(-12.0, 50.0);
}

void runLeftGoal(DriveBase &drive) {

}

void runRightGoal(DriveBase &drive) {
}
