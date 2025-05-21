#include "main.h"
#include "RobotValues.h"
#include "RobotHardware.h"
#include "Auton.h"

pros::Controller master(pros::E_CONTROLLER_MASTER);
RobotHardware    robot(master);

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Press A/B/X to select auton");
  robot.init();
}

void disabled() {}

void competition_initialize() {
  // let kevin pick the auton before match starts
  while (!pros::competition::is_autonomous()) {
    if (master.get_digital_new_press(DIGITAL_A)) {
      selectedAuton = AutonMode::SKILLS;
      pros::lcd::set_text(2, "Auton: Skills");
    }
    if (master.get_digital_new_press(DIGITAL_B)) {
      selectedAuton = AutonMode::LEFT_GOAL;
      pros::lcd::set_text(2, "Auton: Left");
    }
    if (master.get_digital_new_press(DIGITAL_X)) {
      selectedAuton = AutonMode::RIGHT_GOAL;
      pros::lcd::set_text(2, "Auton: Right");
    }
    pros::delay(20);
  }
}

void autonomous() {
  robot.setState(RobotHardware::State::AUTONOMOUS);

  switch (selectedAuton) {
    case AutonMode::SKILLS:     runSkills(robot.driveBase);     break;
    case AutonMode::LEFT_GOAL:  runLeftGoal(robot.driveBase);   break;
    case AutonMode::RIGHT_GOAL: runRightGoal(robot.driveBase);  break;
    default: /* none */;                                   break;
  }
}

void opcontrol() {
  robot.setState(RobotHardware::State::DRIVER);
  while (true) {
    robot.update();
    pros::delay(20);
  }
}
