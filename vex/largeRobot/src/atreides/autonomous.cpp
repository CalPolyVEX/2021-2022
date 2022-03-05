#include "atreides.h"

void at_autonomous() {
	pros::lcd::set_text(0, "Atreides Autonomous");

	// Ensure piston is retracted
	at_clawClosePiston.set_value(0);

	// Deploy claw from reset position to capture position
	at_clawDeployMotor.stepTo(2, 150);
	at_clawDeployMotor.waitUntilSettled();

	std::shared_ptr<okapi::AsyncMotionProfileController> profileController = okapi::AsyncMotionProfileControllerBuilder()
    .withLimits({100.0, 100.0, 100.0})
    .withOutput(at_drivetrain->chassis)
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
        {18_in, 0_ft, 0_deg}},
        "MoveToAWPTeamGoal" // Profile name
  );

	profileController->setTarget("MoveToAWPTeamGoal", true);

	profileController->waitUntilSettled();

	// Close claw
	at_clawClosePiston.set_value(1);

	// Wait for piston to close
	pros::delay(400);

	// Retract claw into scoring position
	at_clawDeployMotor.stepTo(1, 75);

	// While that's happening, start moving towards the rings, and then grab then

	// none of this is actually physically accurate but it kinda works
	// motion profiling just gives up here
	at_clawDeployMotor.waitUntilSettled();
	at_frontIntake.move_voltage(6000);

	at_drivetrain->chassis->moveDistance(12_in);

	for (int i = 0; i < 100; i++) {
		at_drivetrain->chassis->moveDistance(3_in);
		at_drivetrain->chassis->moveDistance(-3_in);
	}

	/*
	at_drivetrain->chassis->turnAngle(100_deg);
	at_drivetrain->chassis->moveDistance(11_in);
	at_drivetrain->chassis->turnAngle(105_deg);
	at_drivetrain->chassis->setMaxVelocity(25);
	at_drivetrain->chassis->moveDistance(36_in);
	at_drivetrain->chassis->setMaxVelocity(1000);*/
}
