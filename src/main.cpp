#include "main.h"
#include "DriveBase.h"
#include "RobotValues.h"
#include "StateMachine.h"
#include "RobotHardware.h"

pros::Controller ctrl(pros::E_CONTROLLER_MASTER);
RobotHardware    robot(ctrl);

void initialize() {
  pros::lcd::initialize();
  pros::lcd::set_text(1, "Hello PROS User!");
  //pros::lcd::register_btn1_cb(on_center_button);
  robot.init();
}

void disabled() {}
void competition_initialize() {}
void autonomous() {}

void opcontrol() {
  while (true) {

    if (ctrl.get_digital(DIGITAL_A))
      robot.setState(RobotHardware::State::IDLE);
    // else if (…) robot.setState(…);

    robot.update();
    pros::delay(20);
  }
}
