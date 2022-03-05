#include "atreides.h"

ControllerButton at_btnResetPos(ControllerDigital::X);
ControllerButton at_btnClawPos1(ControllerDigital::A);
ControllerButton at_btnArmDown(ControllerDigital::L1);
ControllerButton at_btnArmUp(ControllerDigital::L2);
ControllerButton at_btnClaw(ControllerDigital::B);
ControllerButton at_btnIntake1(ControllerDigital::R1);
ControllerButton at_btnIntake2(ControllerDigital::R2);

bool intakeRunning = false;
uint32_t startMillis;

void at_opcontrol_init() {
  pros::lcd::set_text(0, "Atreides Op-Control");

  startMillis = pros::millis();

  // For some reason, motors disengage when switching to opcontrol.
  // Use this as a chance to re-engage them.
}

void at_opcontrol_update() {

  pros::lcd::set_text(1, "BackLeft: " + std::to_string(at_drivetrain->backLeftMotor.getTemperature()) + "*C | BackRight: " + std::to_string(at_drivetrain->backRightMotor.getTemperature()) + "*C");
  pros::lcd::set_text(2, "FrontLeft: " + std::to_string(at_drivetrain->frontLeftMotor.getTemperature()) + "*C");
  pros::lcd::set_text(3, "FrontRight: " + std::to_string(at_drivetrain->frontRightMotor.getTemperature()) + "*C");
  pros::lcd::set_text(4, "BackArmLeft: " + std::to_string(pros::c::motor_get_temperature(5)) + "*C");
  pros::lcd::set_text(5, "BackArmRight: " + std::to_string(pros::c::motor_get_temperature(6)) + "*C");
  pros::lcd::set_text(6, "Intake: " + std::to_string(pros::c::motor_get_temperature(4)) + "*C");
  pros::lcd::set_text(7, "ClawDeploy: " + std::to_string(pros::c::motor_get_temperature(16)) + "*C");


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

  if (at_btnIntake1.changedToPressed() || at_btnIntake2.changedToPressed()) {
    intakeRunning = !intakeRunning;

    if (intakeRunning) {
      at_frontIntake.move_voltage(10000);
    } else {
      at_frontIntake.move_voltage(0);
    }
  }

  uint32_t currentMillis = pros::millis();

  if (currentMillis - startMillis > 73000) {
    // Last two seconds (endgame)

    // Deploy claw from reset position to capture/release position
  	at_clawDeployMotor.stepTo(2, 150);
  	at_clawDeployMotor.waitUntilSettled();

  	// Retract the piston at the end
  	at_clawClosePiston.set_value(0);
  }
}
