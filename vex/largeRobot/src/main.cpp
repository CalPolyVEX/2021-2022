#include "main.h"
#include "robotDriver.h"
#include "arduinoSensors.hpp"
#include "robot_specifics.h"
#include "harkonnen/harkonnen.h"

int ALLOW_TEST_AUTON = 1;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	arduino_sensors_setup();

	pros::lcd::initialize();

	pros::lcd::set_text(0, "Robot Initialized");
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

	ALLOW_TEST_AUTON = 0;
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
	hk_autonomous();
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
	hk_opcontrol_init();

	ArduinoEncoder enc1 = arduino_encoder_create(0);
	ArduinoEncoder enc2 = arduino_encoder_create(1);

#ifdef HAS_MIDDLE_ENCODER
	ArduinoEncoder enc3 = arduino_encoder_create(2);
#endif

	ControllerButton btnTestAuton(ControllerDigital::A);

	while (1) {

#ifdef USE_INTEGRATED_ENCODERS
		pros::lcd::set_text(1, "Using Integrated Encoders");
		pros::lcd::set_text(2, "Arduino encoders disabled in-code");
#else
		pros::lcd::set_text(1, "Encoder 1 Val: " + std::to_string(enc1.get()));
		pros::lcd::set_text(2, "Encoder 2 Val: " + std::to_string(enc2.get()));

#ifdef HAS_MIDDLE_ENCODER
		pros::lcd::set_text(3, "Encoder 3 Val: " + std::to_string(enc3.get()));
#endif
#endif

		hk_opcontrol_update();

		if (btnTestAuton.changedToPressed() && ALLOW_TEST_AUTON) {
			autonomous();
		}

		// Required to avoid us taking up too much time from other tasks.
		pros::delay(20);
	}
}
