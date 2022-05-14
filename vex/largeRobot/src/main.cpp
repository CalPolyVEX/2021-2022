#include "main.h"
#include "CPRobotDriver.hpp"
// #include "arduinoSensors.hpp"

CPRobotDriver *robot = NULL;
CPRobotMotorList *motors = NULL;

/**
 * Runs initialization code. This occurs as soon as the program is started.
 *
 * All other competition modes are blocked by initialize; it is recommended
 * to keep execution time for this mode under a few seconds.
 */
void initialize() {
	motors = new CPRobotMotorList({-2, -3, 4, 15, 17});
	CPRobotMotorSet left({motors->get(2), motors->get(3)});
	CPRobotMotorSet right({motors->get(15), motors->get(17)});
	CPRobotMotorSet lever({motors->get(4)});
	CPRobotControllerBind leverBind(&lever, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_B, 0, std::vector<int> {0, 100, 200, 400, 800, 1600}, Step);
	// CPRobotControllerBind leverBind(&lever, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_B, 0, std::vector<int>{}, Toggle);
	// CPRobotControllerBind leverBind(&lever, pros::E_CONTROLLER_DIGITAL_X, pros::E_CONTROLLER_DIGITAL_B, 0, std::vector<int>{}, Hold);
	std::vector<CPRobotMotorSet *> driveMotors({&left, &right});
	std::vector<CPRobotControllerBind *> binds({&leverBind});
	robot = new CPRobotDriver(driveMotors, TankArcade, binds);

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
	pros::lcd::set_text(0, "Autonomous");
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
	pros::lcd::set_text(0, "Op Control");

	while (1) {
		//Executes the drive code, and any controller binds
		robot->controlCycle();
		// Required to avoid us taking up too much time from other tasks.
		pros::delay(20);
	}
}
