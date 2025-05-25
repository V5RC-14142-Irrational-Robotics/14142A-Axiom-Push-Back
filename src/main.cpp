#include "main.h"
#include "RobotHardware.h"
#include "Auton.h"
#include "Telemetry.h"
#include "pros/llemu.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
RobotHardware    robot(master);
Telemetry        telem;

static std::string getAutonName() {
  switch (selectedAuton) {
    case AutonMode::SKILLS:     return "Skills";
    case AutonMode::LEFT_GOAL:  return "Left";
    case AutonMode::RIGHT_GOAL: return "Right";
    default:                    return "None";
  }
}

void initialize() {
  robot.init();

  telem.clear();
  telem.addLine("ABXY", "A:Skills B:Left X:Right");
  telem.addLine("UP/DN", "Red/Blue");
  telem.addLine("Sel",   getAutonName());
  telem.display();
}

void disabled() {
}

void competition_initialize() {
  while (!pros::competition::is_autonomous()) {
    if (master.get_digital_new_press(DIGITAL_A)) {
      selectedAuton = AutonMode::SKILLS;
    }
    if (master.get_digital_new_press(DIGITAL_B)) {
      selectedAuton = AutonMode::LEFT_GOAL;
    }
    if (master.get_digital_new_press(DIGITAL_X)) {
      selectedAuton = AutonMode::RIGHT_GOAL;
    }
    if (master.get_digital_new_press(DIGITAL_UP)) {
      // set alliance to Red ltr
    }
    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      // set alliance to Blue ltr
    }

    telem.clear();
    telem.addLine("ABXY", "A:Sk B:L X:R");
    telem.addLine("Sel", getAutonName());
    telem.display();

    pros::delay(50);
  }
}

void autonomous() {
  robot.setState(RobotHardware::State::AUTONOMOUS);
  switch (selectedAuton) {
    case AutonMode::SKILLS:     runSkills(robot.driveBase);     break;
    case AutonMode::LEFT_GOAL:  runLeftGoal(robot.driveBase);   break;
    case AutonMode::RIGHT_GOAL: runRightGoal(robot.driveBase);  break;
    default: /* none */;                                        break;
  }
}

void opcontrol() {
  robot.setState(RobotHardware::State::DRIVER);
  while (true) {
    robot.update();
    pros::delay(20);
  }
//runSkills(robot.driveBase);
}