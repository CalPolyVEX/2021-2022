#include "atreides.h"

ControllerButton at_btnResetPos(ControllerDigital::B);
ControllerButton at_btnClawPos1(ControllerDigital::A);
ControllerButton at_btnArmDown(ControllerDigital::L1);
ControllerButton at_btnArmUp(ControllerDigital::L2);
ControllerButton at_btnClaw(ControllerDigital::X);
ControllerButton at_btnIntake(ControllerDigital::Y);

bool intakeRunning = false;

void at_opcontrol_init() {
  pros::lcd::set_text(0, "Atreides Op-Control");

  // For some reason, motors disengage when switching to opcontrol.
  // Use this as a chance to re-engage them.
}

void at_opcontrol_update() {
  at_drivetrain->arcadeDrive();

  if (at_btnClawPos1.changedToPressed()) {
    at_clawDeployMotor.stepExtend();
  } else if (at_btnResetPos.changedToPressed()) {
    at_clawDeployMotor.stepRetract();
  }

  if (at_btnArmDown.changedToPressed()) {
    at_backClawArm->stepExtend();
  } else if (at_btnArmUp.changedToPressed()) {
    at_backClawArm->stepRetract();
  }

  if (at_btnClaw.changedToPressed()) {
		at_clawClosed = !at_clawClosed;
		at_clawClosePiston.set_value(at_clawClosed);
	}

  if (at_btnIntake.changedToPressed()) {
    intakeRunning = !intakeRunning;

    if (intakeRunning) {
      at_frontIntake.move_voltage(6000);
    } else {
      at_frontIntake.move_voltage(0);
    }
  }
}
