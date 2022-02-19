#include "main.h"
#include "robotDriver.h"

#define LEFT_WHEELS_1_PORT 1
#define LEFT_WHEELS_2_PORT 4
#define RIGHT_WHEELS_1_PORT 2
#define RIGHT_WHEELS_2_PORT 3
#define GYRO_PORT 10

#define WHEEL_RADIUS 4

#define ENCODER_COUNT 3
#define ENCODER_PPR 100

//RobotDriver
RobotDriver *robo = new RobotDriver(LEFT_WHEELS_1_PORT, RIGHT_WHEELS_1_PORT, LEFT_WHEELS_2_PORT, RIGHT_WHEELS_2_PORT, GYRO_PORT, WHEEL_RADIUS);

/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {
	static bool pressed = false;
	pressed = !pressed;
	if (pressed) {
		pros::lcd::set_text(2, "I was pressed!");
	} else {
		pros::lcd::clear_line(2);
	}
}

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	robo->configTurnPID(3, 0.0001, 0, 10, 18, 128);
	robo->configPositionPID(65, 0, 0, 10);
	robo->configEncoders(ENCODER_COUNT, ENCODER_PPR);

	pros::lcd::initialize();

	pros::lcd::set_text(0, "Robot Initialized");

	pros::lcd::register_btn1_cb(on_center_button);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {
	pros::lcd::set_text(0, "Robot Disabled");
}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {
	pros::lcd::set_text(0, "Competition Initialized");
}

/**
 * Runs the user autonomous code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the autonomous
 * mode. Alternatively, this function may be called in initialize or opcontrol
 * for non-competition testing purposes.
 *
 * If the robot is disabled or communications is lost, the autonomous task
 * will be stopped. Re-enabling the robot will restart the task, not re-start it
 * from where it left off.
 */

void autonomous() {
	pros::lcd::set_text(1, "Autonomous");
	pros::delay(500);
}

/**
 * Runs the operator control code. This function will be started in its own task
 * with the default priority and stack size whenever the robot is enabled via
 * the Field Management System or the VEX Competition Switch in the operator
 * control mode.
 *
 * If no competition control is connected, this function will run immediately
 * following initialize().
 *
 * If the robot is disabled or communications is lost, the
 * operator control task will be stopped. Re-enabling the robot will restart the
 * task, not resume it from where it left off.
 */

void opcontrol() {
	pros::lcd::set_text(0, "Op-Control");
	while (1) {
	  robo->tankDrive();

		pros::lcd::set_text(1, "Encoder 1 Val: " + std::to_string(robo->readEncoder(1)));
		pros::lcd::set_text(2, "Encoder 2 Val: " + std::to_string(robo->getEncoderVal(2)));
		pros::lcd::set_text(3, "Encoder 3 Val: " + std::to_string(robo->getEncoderVal(3)));

		if (robo->getController()->get_digital(DIGITAL_B) == 1) {
			// robo->positionPID(20);
		}
		if (robo->getController()->get_digital(DIGITAL_X) == 1) {
			robo->turnPID(-90);
		}
		if (robo->getController()->get_digital(DIGITAL_Y) == 1) {
			robo->turnPIDAndRecalibrate(90);
		}
		//delay to save resources
		pros::delay(20);
	}
}


//Random commented out code in case we'd like to reference anything:


//open/close claw based on left triggers
// if (master.get_digital(DIGITAL_L1)) {
//   claw.move_velocity(100);
// }
// else if (master.get_digital(DIGITAL_L2)) {
//   claw.move_velocity(-100);
// }
// else {
//   claw.move_velocity(0);
// }
//toggle pistons with touch button
// if (touch_button.get_value()) {
// 	piston_1.set_value(true);
// 	piston_2.set_value(true);
// }
// else {
// 	piston_1.set_value(false);
// 	piston_2.set_value(false);
// }
