#ifndef AUTON_H
#define AUTON_H

#include "DriveBase.h"

/// Which auton the driver has selected
enum class AutonMode {
  NONE,
  SKILLS,
  LEFT_GOAL,
  RIGHT_GOAL
};

extern AutonMode selectedAuton;

void runSkills(DriveBase &drive);
void runLeftGoal(DriveBase &drive);
void runRightGoal(DriveBase &drive);

#endif
