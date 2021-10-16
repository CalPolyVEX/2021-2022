#include "main.h"

#define ACCELEROMETER_X 15
#define ACCELEROMETER_Y 15
#define ACCELEROMETER_Z 15

pros::Imu imu_sensor(15);
/**
 * A callback function for LLEMU's center button.
 *
 * When this callback is fired, it will toggle line 2 of the LCD text between
 * "I was pressed!" and nothing.
 */
void on_center_button() {}

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
 * task, not resume it from where it left off
 */






void opcontrol() {

	pros::Controller master(pros::E_CONTROLLER_MASTER);
	pros::Motor rear_left_mtr(4);
	pros::Motor rear_right_mtr(3);
	pros::Motor front_right_mtr(2);
	pros::Motor front_left_mtr(1);

	pros::Motor claw_mtr(13);

	while (true) {

		rear_left_mtr.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		rear_right_mtr.move(-master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
		front_left_mtr.move(master.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
		front_right_mtr.move(-master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));

		claw_mtr.move(master.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X));


		//std::string YAW = std::to_string(imu_sensor.get_yaw());
		//pros::lcd::set_text(1, YAW);

		//pros::screen::print(pros::E_TEXT_LARGE, 3, "Value");


		if (master.get_digital(DIGITAL_X)==1) {

			while (imu_sensor.get_yaw() < 30 && imu_sensor.get_yaw() > -30) {
				//double yaw1 = imu_sensor.get_yaw();
				//std::string YAW = std::to_string(imu_sensor.get_yaw());
				//pros::lcd::set_text(1, YAW);

		 			rear_left_mtr = 30;
		 			front_left_mtr = 30;
		 			rear_right_mtr = 30;
		 			front_right_mtr = 30;

				}

				rear_left_mtr = 0;
				front_left_mtr = 0;
				rear_right_mtr = 0;
				front_right_mtr = 0;

		 		}
		 	}
		}
