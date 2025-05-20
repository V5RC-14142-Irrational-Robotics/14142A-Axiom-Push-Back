#include "main.h"

//initializing constructors

pros::Controller ctrl(pros::E_CONTROLLER_MASTER);
pros::MotorGroup driveLeft({1, -2, 3}, pros::v5::MotorGears::blue);
pros::MotorGroup driveRight({-4, 5, -6}, pros::v5::MotorGears::blue);

pros::Imu imu(7);

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	pros::lcd::initialize();
	pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);
}

//disable state following either auton or driver control
void disabled() {}

//after initialize and before auton, used for auton selectors
void competition_initialize() {}

//auton
void autonomous() {}

//driver control
void opcontrol() {
	while (true) {
		//2-stick arcade drive
		int pwr = ctrl.get_analog(ANALOG_LEFT_Y);
		int turn = ctrl.get_analog(ANALOG_RIGHT_X);
		int left = pwr + turn;
		int right = pwr - turn;
		driveLeft.move(left);
		driveRight.move(right);
	}
}