#include "atreides.h"

void at_autonomous() {
	pros::lcd::set_text(0, "Atreides Autonomous");

	/*#ifdef AUTONOMOUS_ONLY_DRIVE_FORWARD
		hk_drivetrain->chassis->setMaxVelocity(100000);
		hk_drivetrain->chassis->moveDistance(1000_m);

		return;
	#endif

	std::shared_ptr<okapi::AsyncMotionProfileController> profileController = okapi::AsyncMotionProfileControllerBuilder()
    .withLimits({100.0, 100.0, 100.0})
    .withOutput(hk_drivetrain->chassis)
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

	// Claw in released position
	// Lower back arm to ground position.
  hk_backArm->stepTo(3, 5000);

  profileController->waitUntilSettled();
	hk_backArm->waitUntilSettled();

	// Back arm is now lowered, hold the goal in the claw.
	hk_clamp->extend();

	// This seems to not deadlock, even though it clamps down hard on the goal.
	hk_clamp->waitUntilSettled();

	// Raise back arm to raised position.
  hk_backArm->stepTo(2, 5000);

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
  profileController->waitUntilSettled();*/
}
