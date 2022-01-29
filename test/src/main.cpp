#include "main.h"
#include "robotDriver.h"

#define LEFT_WHEELS_1_PORT 1
#define LEFT_WHEELS_2_PORT 4
#define RIGHT_WHEELS_1_PORT 2
#define RIGHT_WHEELS_2_PORT 3
#define CLAW_PORT 13
#define GYRO_PORT 15
#define TOUCH_BUTTON_PORT 'A'
#define PISTON_1_PORT 'B'
#define PISTON_2_PORT 'C'

#define ERROR_BOUND_DRIVE 0.001
#define ERROR_BOUND_TURN 0.1

//RobotDriver
RobotDriver *robo = new RobotDriver(LEFT_WHEELS_1_PORT, RIGHT_WHEELS_1_PORT, LEFT_WHEELS_2_PORT, RIGHT_WHEELS_2_PORT, GYRO_PORT, 4);


//controller
pros::Controller master(pros::E_CONTROLLER_MASTER);
//drive motors
// pros::Motor left_mtr_1(LEFT_WHEELS_1_PORT);
// pros::Motor left_mtr_2(LEFT_WHEELS_2_PORT);
// pros::Motor right_mtr_1(RIGHT_WHEELS_1_PORT);
// pros::Motor right_mtr_2(RIGHT_WHEELS_2_PORT);
//claw motor
// pros::Motor claw (CLAW_PORT, MOTOR_GEARSET_36);
//pistons
// pros::ADIDigitalOut piston_1 (PISTON_1_PORT);
// pros::ADIDigitalOut piston_2 (PISTON_2_PORT);
//touch touch sensor
// pros::ADIDigitalIn touch_button (TOUCH_BUTTON_PORT);
//gyro
// pros::Imu gyro (GYRO_PORT);

/**
Utility functions
*/
// double compare_rotations(double rot_i, double rot_f, int dir) {
// 	double difference = rot_f - rot_i;
// 	while (difference * dir < 0) {
// 		difference += 360 * dir;
// 	}
// 	return difference;
// }

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

// double clamp(double val, double max, double min) {
// 	double sign = 1;
// 	if (val < 0) {
// 		sign = -1;
// 	}
// 	if (abs(val) < min) {
// 		return min * sign;
// 	} else if (abs(val) > max) {
// 		return max * sign;
// 	} else {
// 		return val;
// 	}
// }

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	robo->configTurnPID(2.5, 0, 0, 10);
	robo->configPositionPID(65, 0, 0, 10);


	pros::lcd::initialize();
	//pros::lcd::set_text(1, "Hello PROS User!");

	pros::lcd::register_btn1_cb(on_center_button);

	//pros::ADIDigitalOut piston (DIGITAL_SENSOR_PORT);
}

/**
 * Runs while the robot is in the disabled state of Field Management System or
 * the VEX Competition Switch, following either autonomous or opcontrol. When
 * the robot is enabled, this task will exit.
 */
void disabled() {}

/**
 * Runs after initialize(), and before autonomous when connected to the Field
 * Management System or the VEX Competition Switch. This is intended for
 * competition-specific initialization routines, such as an autonomous selector
 * on the LCD.
 *
 * This task will exit when the robot is enabled and autonomous or opcontrol
 * starts.
 */
void competition_initialize() {}

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

void autonomous() {}

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
	while (1) {
	  //get joystick values, and use those values to drive
		// int left = master.get_analog(ANALOG_LEFT_Y);
		// int right = -master.get_analog(ANALOG_RIGHT_Y);
		// left_mtr_1 = left;
		// left_mtr_2 = left;
		// right_mtr_1 = right;
		// right_mtr_2 = right;

		robo->encoderTest();

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

		if (master.get_digital(DIGITAL_B) == 1) {
			// robo->positionPID(20);
		}
		if (master.get_digital(DIGITAL_X) == 1) {
			// robo->turnPID(-90);
		}

		//delay to save resources
		pros::delay(20);
	}
}
