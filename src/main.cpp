#include "main.h"
#include "RobotValues.h"
#include "RobotHardware.h"
#include "Auton.h"
#include "liblvgl/llemu.h"
#include "liblvgl/llemu.hpp"

pros::Controller master(pros::E_CONTROLLER_MASTER);
RobotHardware    robot(master);

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(0, "Initialized");
  pros::lcd::set_text(1, "Press A/B/X to select auton");
  pros::lcd::set_text(2, "Press UP/DOWN to select color");
  robot.init();
}

void disabled() {pros::lcd::set_text(0, "Disabled");}

void competition_initialize() {
  pros::lcd::set_text(0, "Competition Initialized");
  while (!pros::competition::is_autonomous()) {
    //auton routine selection
    if (master.get_digital_new_press(DIGITAL_A)) {
      selectedAuton = AutonMode::SKILLS;
      pros::lcd::set_text(4, "Auton: Skills");
      master.set_text(0, 0, "Auton: Skills");
      master.rumble(".");
    }
    if (master.get_digital_new_press(DIGITAL_B)) {
      selectedAuton = AutonMode::LEFT_GOAL;
      pros::lcd::set_text(4, "Auton: Left");
      master.set_text(0, 0, "Auton: Left");
      master.rumble(".");
    }
    if (master.get_digital_new_press(DIGITAL_X)) {
      selectedAuton = AutonMode::RIGHT_GOAL;
      pros::lcd::set_text(4, "Auton: Right");
      master.set_text(0, 0, "Auton: Right");
      master.rumble(".");
    }

    //potential color sort selection
    if (master.get_digital_new_press(DIGITAL_UP)) {
      //pros::lcd::set_background_color(245, 66, 66);
      pros::lcd::set_text(5, "Red Alliance");
      master.set_text(1, 0, "Red Alliance");
      master.rumble(".");
    }
    if (master.get_digital_new_press(DIGITAL_DOWN)) {
      //pros::lcd::set_background_color(66, 135, 245);
      pros::lcd::set_text(5, "Blue Alliance");
      master.set_text(1, 0, "Blue Alliance");
      master.rumble(".");
    }
    
    pros::delay(20);
  }
}

void autonomous() {
  robot.setState(RobotHardware::State::AUTONOMOUS);
  pros::lcd::set_text(0, "Autonomous");

  switch (selectedAuton) {
    case AutonMode::SKILLS:     runSkills(robot.driveBase);     break;
    case AutonMode::LEFT_GOAL:  runLeftGoal(robot.driveBase);   break;
    case AutonMode::RIGHT_GOAL: runRightGoal(robot.driveBase);  break;
    default: /* none */;                                        break;
  }
}

void opcontrol() {
  robot.setState(RobotHardware::State::DRIVER);
  pros::lcd::set_text(0, "Driver Control");
  while (true) {
    robot.update();
    pros::delay(20);
  }
}
