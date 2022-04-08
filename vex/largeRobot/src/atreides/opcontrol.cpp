#include "atreides.h"

ControllerButton at_btnResetPos(ControllerDigital::X);
ControllerButton at_btnClawPos1(ControllerDigital::A);
ControllerButton at_btnArmUp(ControllerDigital::L1);
ControllerButton at_btnArmDown(ControllerDigital::L2);
ControllerButton at_btnClaw(ControllerDigital::B);
ControllerButton at_btnIntake(ControllerDigital::R1);
ControllerButton at_btnIntake2(ControllerDigital::R2);

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

	// arm lifting has been commented out
  //if (at_btnArmDown.isPressed()) {
  //  Arm1.move_voltage(12000);
  //  Arm2.move_voltage(-12000);
  //} else if (at_btnArmUp.isPressed()) {
  //  Arm1.move_voltage(-12000);
   // Arm2.move_voltage(12000);
  //}
  //else{
    //  Arm1.move_voltage(0);
    //  Arm2.move_voltage(0);
  //}

  if (at_btnClaw.changedToPressed()) {
		at_clawClosed = !at_clawClosed;
		at_clawClosePiston.set_value(at_clawClosed);
	}

  if (at_btnIntake.changedToPressed() || at_btnIntake2.changedToPressed()) {
    intakeRunning = !intakeRunning;

    if (intakeRunning) {
      at_frontIntake.move_voltage(10000);
    } else {
      at_frontIntake.move_voltage(0);
    }
  }
}
