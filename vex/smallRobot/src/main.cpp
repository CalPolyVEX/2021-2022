#include "main.h"
#include "robotDriver.h"
#include "arduinoSensors.hpp"
#include "robot_specifics.h"

#define LEFT_WHEELS_1_PORT 2
#define LEFT_WHEELS_2_PORT 3
#define RIGHT_WHEELS_1_PORT 17
#define RIGHT_WHEELS_2_PORT 18
#define PISTON_PORT 'A'

//RobotDriver
RobotDriver *robo = new RobotDriver(LEFT_WHEELS_1_PORT, RIGHT_WHEELS_1_PORT, LEFT_WHEELS_2_PORT, RIGHT_WHEELS_2_PORT);

#define INTAKE_PORT 4
#define ARM_LEFT 5
#define ARM_RIGHT 6 //move together
#define CLAW_PORT 16

// pros::Motor flLever(FRONT_LEVER_LEFT_PORT);
// pros::Motor frLever(FRONT_LEVER_RIGHT_PORT);
// pros::Motor blLever(BACK_LEVER_LEFT_PORT);
// pros::Motor brLever(BACK_LEVER_RIGHT_PORT);
pros::Motor claw(CLAW_PORT);
pros::ADIDigitalOut piston (PISTON_PORT);

// std::shared_ptr<okapi::AsyncPositionController<double, double>> frontArm =
// 	okapi::AsyncPosControllerBuilder().withMotor({FRONT_LEVER_LEFT_PORT, -FRONT_LEVER_RIGHT_PORT})
// 	.build();
//
std::shared_ptr<okapi::AsyncPositionController<double, double>> Arm =
	okapi::AsyncPosControllerBuilder().withMotor({ARM_LEFT, -ARM_RIGHT})
	.build();

std::shared_ptr<okapi::AsyncPositionController<double, double>> clawCtl =
		okapi::AsyncPosControllerBuilder().withMotor(CLAW_PORT)
		.build();

int ALLOW_TEST_AUTON = 1;

//

const int NUM_CLAWPOS_HEIGHTS = 3;
const int frontHeights[NUM_CLAWPOS_HEIGHTS] = {
	0,
	-2000,
	-200
};

const int NUM_ARM_HEIGHTS = 4;
const int backHeights[NUM_ARM_HEIGHTS] = {
	0,
	-2000,
	-4000,
	-6000
};

int frontGoalHeight = 0;
int backGoalHeight = 0;
bool clawHold = false;

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
        {-54_in, 0_ft, 0_deg}},
        "MoveToMiddleGoal" // Profile name
  );

	profileController->setTarget("MoveToMiddleGoal", true);

	// // Claw in released position
	// // Lower back arm to ground position.
	// backGoalHeight = 3;
	// backArm->setTarget(backHeights[backGoalHeight]);
	//
  // profileController->waitUntilSettled();
	// backArm->waitUntilSettled();
	//
	// // Back arm is now lowered, hold the goal in the claw.
	// clawCtl->setTarget(CLAW_HOLD_TARGET);
	//
	// // This seems to not deadlock, even though it clamps down hard on the goal.
	// clawCtl->waitUntilSettled();
	//
	// // Raise back arm to raised position.
	// backGoalHeight = 0;
	// backArm->setTarget(backHeights[backGoalHeight]);
	// // NB: Don't wait before moving.

	// Start moving without waiting.
	profileController->generatePath({
        {0_in, 0_in, 0_deg},
        {-60_in, 0_ft, 0_deg}},
        "ReturnFromMiddleGoal" // Profile name
  );

	// flip this boolean to move backwards instead of forwards
	// apparently the underlying pathfinder library can't generate
	// negative velocities / backwards paths, so this is the workaround.
	profileController->setTarget("ReturnFromMiddleGoal", false);
  profileController->waitUntilSettled();


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
	// Ensure that we keep the same state we had in auton
	// For some reason motors the motors disengaged when switching to auton.
	// No idea why, but I don't have time to find out why right now
	// clawHold = 1;
	// backGoalHeight = 0;
	// clawCtl->setTarget(CLAW_HOLD_TARGET);
	// backArm->setTarget(backHeights[backGoalHeight]);

	pros::lcd::set_text(0, "Op-Control");
	pros::Controller *ctrl = robo->getController();

	ArduinoEncoder enc1 = arduino_encoder_create(0);
	ArduinoEncoder enc2 = arduino_encoder_create(1);

#ifdef HAS_MIDDLE_ENCODER
	ArduinoEncoder enc3 = arduino_encoder_create(2);
#endif

	ControllerButton btnTestAuton(ControllerDigital::X);
	ControllerButton btnResetPos(ControllerDigital::B);
	ControllerButton btnClawPos1(ControllerDigital::A);
	ControllerButton btnArmDown(ControllerDigital::L1);
	ControllerButton btnArmUp(ControllerDigital::L2);
	ControllerButton btnClaw(ControllerDigital::X);
	//ControllerButton btnClawRelease(ControllerDigital::Y);

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
		if (btnClawPos1.changedToPressed()) {
      // If the goal height is not at maximum and the up button is pressed, increase the setpoint
      frontGoalHeight = 1;
			clawCtl->setMaxVelocity(75);
      clawCtl->setTarget(frontHeights[1]);
    } else if (btnResetPos.changedToPressed()) {
      frontGoalHeight = 0;
			clawCtl->setMaxVelocity(75);
      clawCtl->setTarget(frontHeights[0]);
    }

		// back
		if (btnArmDown.changedToPressed() && backGoalHeight < NUM_ARM_HEIGHTS - 1) {
      // If the goal height is not at maximum and the up button is pressed, increase the setpoint
      backGoalHeight++;
      Arm->setTarget(backHeights[backGoalHeight]);
    } else if (btnArmUp.changedToPressed() && backGoalHeight > 0) {
      backGoalHeight--;
      Arm->setTarget(backHeights[backGoalHeight]);
    }

		if (btnClaw.changedToPressed()) {
			clawHold = !clawHold;
			piston.set_value(clawHold);
		}

		if (btnTestAuton.changedToPressed() && ALLOW_TEST_AUTON) {
			autonomous();
		}

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
