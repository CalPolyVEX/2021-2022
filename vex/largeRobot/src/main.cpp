#include "main.h"
#include "robotDriver.h"
#include "arduinoSensors.hpp"
#include "robot_specifics.h"

#define LEFT_WHEELS_1_PORT 1
#define LEFT_WHEELS_2_PORT 4
#define RIGHT_WHEELS_1_PORT 2
#define RIGHT_WHEELS_2_PORT 3

//RobotDriver
RobotDriver *robo = new RobotDriver(LEFT_WHEELS_1_PORT, RIGHT_WHEELS_1_PORT, LEFT_WHEELS_2_PORT, RIGHT_WHEELS_2_PORT);

#define FRONT_LEVER_LEFT_PORT 15
#define FRONT_LEVER_RIGHT_PORT 16
#define BACK_LEVER_LEFT_PORT 12
#define BACK_LEVER_RIGHT_PORT 13
#define CLAW_PORT 19

pros::Motor flLever(FRONT_LEVER_LEFT_PORT);
pros::Motor frLever(FRONT_LEVER_RIGHT_PORT);
pros::Motor blLever(BACK_LEVER_LEFT_PORT);
pros::Motor brLever(BACK_LEVER_RIGHT_PORT);
pros::Motor claw(CLAW_PORT);

std::shared_ptr<okapi::AsyncPositionController<double, double>> frontArm =
	okapi::AsyncPosControllerBuilder().withMotor({FRONT_LEVER_LEFT_PORT, -FRONT_LEVER_RIGHT_PORT})
	.build();

std::shared_ptr<okapi::AsyncPositionController<double, double>> backArm =
	okapi::AsyncPosControllerBuilder().withMotor({BACK_LEVER_LEFT_PORT, -BACK_LEVER_RIGHT_PORT})
	.build();

std::shared_ptr<okapi::AsyncPositionController<double, double>> clawCtl =
		okapi::AsyncPosControllerBuilder().withMotor(CLAW_PORT)
		.build();

int ALLOW_TEST_AUTON = 1;

//

const int NUM_FRONT_HEIGHTS = 4;
const int frontHeights[NUM_FRONT_HEIGHTS] = {
	0,
	-500,
	-1000,
	-1450
};

const int NUM_BACK_HEIGHTS = 4;
const int backHeights[NUM_BACK_HEIGHTS] = {
	0,
	1000,
	2000,
	3000
};

int frontGoalHeight = 0;
int backGoalHeight = 0;
int clawHold = 0;

#define CLAW_HOLD_TARGET (-260)
#define CLAW_RELEASED_TARGET 0

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
	arduino_sensors_setup();

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
	pros::lcd::set_text(0, "Autonomous");

	std::shared_ptr<okapi::AsyncMotionProfileController> profileController = okapi::AsyncMotionProfileControllerBuilder()
    .withLimits({100.0, 100.0, 100.0})
    .withOutput(robo->chassis)
    .buildMotionProfileController();

	// Generate an "S-curve" that takes us through the given waypoints.
	// S-curves let us move as accurate as we can, instead of repeatedly doing
	// "turn" and then "move straight".
	//
	// You can still do them as normal, but you have to make sure that the
	// profile controller is settled first.

	// This might overshoot the target a bit but that's OK.
	// We'd rather overshoot than undershoot. If we undershoot
	// then we don't get anything. If we overshoot then we'll
	// push the goal a bit but we can still get it.
	profileController->generatePath({
        {0_in, 0_ft, 0_deg},
        {54_in, 0_ft, 0_deg}},
        "MoveToMiddleGoal" // Profile name
  );

	profileController->setTarget("MoveToMiddleGoal", true);

	// Claw in released position
	// Lower back arm to ground position.
	backGoalHeight = 3;
	backArm->setTarget(backHeights[backGoalHeight]);

  profileController->waitUntilSettled();
	backArm->waitUntilSettled();

	// Back arm is now lowered, hold the goal in the claw.
	clawCtl->setTarget(CLAW_HOLD_TARGET);

	// This seems to not deadlock, even though it clamps down hard on the goal.
	clawCtl->waitUntilSettled();

	// Raise back arm to raised position.
	backGoalHeight = 1;
	backArm->setTarget(backHeights[backGoalHeight]);
	// NB: Don't wait before moving.

	// Start moving without waiting.
	// TODO: Write this path based on what we actually need
	/*profileController->generatePath({
        {-58_in, 0_in, 0_deg},
        {0_ft, 0_ft, 0_deg}},
        "ReturnFromMiddleGoal" // Profile name
  );

	profileController->setTarget("ReturnFromMiddleGoal", true);
  profileController->waitUntilSettled();*/


	// Lower front arm to ground position.
	//frontGoalHeight = 3;
	//frontArm->setTarget(frontHeights[frontGoalHeight]);
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
	pros::Controller *ctrl = robo->getController();

	ArduinoEncoder enc1 = arduino_encoder_create(0);
	ArduinoEncoder enc2 = arduino_encoder_create(1);

#ifdef HAS_MIDDLE_ENCODER
	ArduinoEncoder enc3 = arduino_encoder_create(2);
#endif

	ControllerButton btnTestAuton(ControllerDigital::A);

	ControllerButton btnFrontUp(ControllerDigital::R1);
	ControllerButton btnFrontDown(ControllerDigital::R2);
	ControllerButton btnBackUp(ControllerDigital::L1);
	ControllerButton btnBackDown(ControllerDigital::L2);
	ControllerButton btnClawHold(ControllerDigital::X);
	ControllerButton btnClawRelease(ControllerDigital::Y);

	while (1) {

#if DRIVE_MODE == TANK
	  robo->tankDrive();
#else
	  robo->arcadeDrive();
#endif

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

		//front arm
		if (btnFrontDown.changedToPressed() && frontGoalHeight < NUM_FRONT_HEIGHTS - 1) {
      // If the goal height is not at maximum and the up button is pressed, increase the setpoint
      frontGoalHeight++;
			frontArm->setMaxVelocity(5000);
      frontArm->setTarget(frontHeights[frontGoalHeight]);
    } else if (btnFrontUp.changedToPressed() && frontGoalHeight > 0) {
      frontGoalHeight--;
			frontArm->setMaxVelocity(50);
      frontArm->setTarget(frontHeights[frontGoalHeight]);
    }

		// back
		if (btnBackDown.changedToPressed() && backGoalHeight < NUM_BACK_HEIGHTS - 1) {
      // If the goal height is not at maximum and the up button is pressed, increase the setpoint
      backGoalHeight++;
      backArm->setTarget(backHeights[backGoalHeight]);
    } else if (btnBackUp.changedToPressed() && backGoalHeight > 0) {
      backGoalHeight--;
      backArm->setTarget(backHeights[backGoalHeight]);
    }

		if (btnClawHold.changedToPressed() && !clawHold) {
			clawHold = 1;
			clawCtl->setTarget(CLAW_HOLD_TARGET);
		} else if (btnClawRelease.changedToPressed() && clawHold) {
			clawHold = 0;
			clawCtl->setTarget(CLAW_RELEASED_TARGET);
		}

		if (btnTestAuton.changedToPressed() && ALLOW_TEST_AUTON) {
			autonomous();
		}

		//back arm
		/*if (ctrl->get_digital(DIGITAL_L1)) {
			blLever = -96;
			brLever = 96;
		} else if (ctrl->get_digital(DIGITAL_L2)) {
			blLever = 96;
			brLever = -96;
		} else {
			blLever = 0;
			brLever = 0;
		}*/

		//claw
		/*if (ctrl->get_digital(DIGITAL_X)){
			claw = -100;
		}
		else if(ctrl->get_digital(DIGITAL_Y)){
			claw = 100;
		}*/

		// delay to save resources
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
