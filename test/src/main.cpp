#include "main.h"

#define LEFT_WHEELS_1_PORT 1
#define LEFT_WHEELS_2_PORT 4
#define RIGHT_WHEELS_1_PORT 2
#define RIGHT_WHEELS_2_PORT 3
#define CLAW_PORT 13
#define GYRO_PORT 15
#define TOUCH_BUTTON_PORT 'A'
#define PISTON_1_PORT 'B'
#define PISTON_2_PORT 'C'

#define ERROR_BOUND 1

//controller
pros::Controller master(pros::E_CONTROLLER_MASTER);
//drive motors
pros::Motor left_mtr_1(LEFT_WHEELS_1_PORT);
pros::Motor left_mtr_2(LEFT_WHEELS_2_PORT);
pros::Motor right_mtr_1(RIGHT_WHEELS_1_PORT);
pros::Motor right_mtr_2(RIGHT_WHEELS_2_PORT);
//claw motor
pros::Motor claw (CLAW_PORT, MOTOR_GEARSET_36);
//pistons
pros::ADIDigitalOut piston_1 (PISTON_1_PORT);
pros::ADIDigitalOut piston_2 (PISTON_2_PORT);
//touch touch sensor
pros::ADIDigitalIn touch_button (TOUCH_BUTTON_PORT);
//gyro
pros::Imu gyro (GYRO_PORT);

std::shared_ptr<okapi::ChassisController> chassis =
  okapi::ChassisControllerBuilder()
	.withMotors({LEFT_WHEELS_1_PORT, LEFT_WHEELS_2_PORT}, {-RIGHT_WHEELS_1_PORT, -RIGHT_WHEELS_2_PORT})
    // Green gearset, 4 in wheel diam, 11.5 in wheel track
    .withDimensions(AbstractMotor::gearset::green, {{4.1_in, 14.6_in}, imev5GreenTPR})
    .build();

/**
Utility functions
*/
double compare_rotations(double rot_i, double rot_f, int dir) {
	double difference = rot_f - rot_i;
	while (difference * dir < 0) {
		difference += 360 * dir;
	}
	return difference;
}

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
 void compareFunc() {
	 while (true) {
		 double output = 127;
		 left_mtr_1 = output;
		 left_mtr_2 = output;
		 right_mtr_1 = -output;
		 right_mtr_2 = -output;
	 }
 }

 void positionPID(double desired_dist) {
	 double error_prior = 0;
	 double integral_prior = 0;
	 double dT = 10;
	 double kP = 127;
	 double kD = 10;
	 double kI = 5;

	 // shoutout Kyle for PI
	 // desired revolutions
	 // circumference units = inches
	 double circumference = (4*3.14159265358979323846264338327950288419716939);
	 // desired units = revolutions
	 double desired = desired_dist/circumference;
	 left_mtr_1.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	 // initActual units = revolutions
	 double initialActual = left_mtr_1.get_position();


	 while (true) {
		 // actual units = revolutions
		 double actual = (left_mtr_1.get_position()) - initialActual;
		 // error units = revolutions
		 double error = desired - actual;
		 pros::lcd::set_text(2, "Revs: " + std::to_string(left_mtr_1.get_position()));
		 pros::lcd::set_text(3, "Act: " + std::to_string(actual));
		 pros::lcd::set_text(4, "Err: " + std::to_string(error));
		 if (abs(error) < ERROR_BOUND) {
			 break;
		 }
		 double integral = integral_prior - (error*dT);
		 double derivative = (error - error_prior)/dT;
		 // TODO: What do we do with ouput? How should we llink output to motors?
		 int8_t output = (int8_t)((kP*error) + (kI*integral) + (kD*derivative));
		 pros::lcd::set_text(5, "Output: " + std::to_string(output));
		 if (error > 0) {
			 output = 30;
		 } else {
			 output = -30;
		 }
		 //double output = 15;
		 left_mtr_1 = output;
		 left_mtr_2 = output;
		 right_mtr_1 = -output;
		 right_mtr_2 = -output;

		 error_prior = error;
		 integral_prior = integral;
		 pros::delay(dT);
	 }
 }

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
	//gyro.reset();
	int turning = 0;
	double pre_turn_rotation = gyro.get_yaw();

	while (true) {
	  //get joystick values, and use those values to drive
		int left = master.get_analog(ANALOG_LEFT_Y);
		int right = -master.get_analog(ANALOG_RIGHT_Y);
		// left_mtr_1 = left;
		// left_mtr_2 = left;
		// right_mtr_1 = right;
		// right_mtr_2 = right;
		//open/close claw based on left triggers
		if (master.get_digital(DIGITAL_L1)) {
      claw.move_velocity(100);
    }
    else if (master.get_digital(DIGITAL_L2)) {
      claw.move_velocity(-100);
    }
    else {
      claw.move_velocity(0);
    }
		//toggle pistons with touch button
		// if (touch_button.get_value()) {
		// 	piston_1.set_value(true);
		// 	piston_2.set_value(true);
    // }
    // else {
		// 	piston_1.set_value(false);
		// 	piston_2.set_value(false);
    // }
		// //determine whether turning
		// if (turning == 0 && master.get_digital(DIGITAL_A) == 1) {
		// 	pre_turn_rotation = gyro.get_yaw();
		// 	turning = 1;
		// } else if (turning == 0 && master.get_digital(DIGITAL_Y) == 1) {
		// 	pre_turn_rotation = gyro.get_yaw();
		// 	turning = -1;
		// }

		if (master.get_digital(DIGITAL_B) == 1) {
			// compareFunc();
			// positionPID(30);
			// Move 1 meter to the first goal
			chassis->setMaxVelocity(100);
			chassis->moveDistance(20_in);
      pros::delay(300);
// // Turn 90 degrees to face second goal
      chassis->turnAngle(90_deg);

      pros::delay(300);
// // Drive 1 and a half feet toward second goal

// 			chassis->moveDistance(5_in);



		}

		// double current_rotation = gyro.get_yaw();
		// double rotation_difference = compare_rotations(pre_turn_rotation, current_rotation, turning);
		// double turnAmount = 90.0;
		// if (turning != 0 && std::abs(rotation_difference) < turnAmount) {
		// 	double throttle = 0;
		// 	/*
		// 	This spaghetti code throttles the turning speed
		// 	It looks at the remaining amount of turning required and scales the throttle amount
		// 	*/
		// 	if (turnAmount - std::abs(rotation_difference) < turnAmount * 0.6) {
		// 		throttle = ((25 - 15) / (turnAmount * 0.6)) * ((turnAmount * 0.6) - (turnAmount - std::abs(rotation_difference)));
		// 	}
		// 	left_mtr_1 = (25 - throttle) * turning;
		// 	left_mtr_2 = (25 - throttle) * turning;
		// 	right_mtr_1 = (25 - throttle) * turning;
		// 	right_mtr_2 = (25 - throttle) * turning;
		// } else {
		// 	turning = 0;
		// }
		//print stuff
		double gyroVal = gyro.get_yaw();
		pros::lcd::set_text(1, "Opcontrol loop"); //std::to_string(rotation_difference)
		//delay to save resources
		pros::delay(20);
	}
}
