#include "harkonnen.h"

ControllerButton hk_btnFrontUp(ControllerDigital::R1);
ControllerButton hk_btnFrontDown(ControllerDigital::R2);
ControllerButton hk_btnBackUp(ControllerDigital::L1);
ControllerButton hk_btnBackDown(ControllerDigital::L2);
ControllerButton hk_btnClampToggle(ControllerDigital::X);

void hk_opcontrol_init() {
  pros::lcd::set_text(0, "Op-Control");

  // For some reason, motors disengage when switching to opcontrol.
  hk_clamp->engage();
  hk_backArm->engage();

  pros::c::motor_set_brake_mode(MIDDLE_WACKO_WHEEL_PORT, pros::E_MOTOR_BRAKE_BRAKE);
}

void hk_opcontrol_update() {
  pros::lcd::set_text(1, "BackLeft: " + std::to_string(hk_drivetrain->backLeftMotor.getTemperature()) + "*C | BackRight: " + std::to_string(hk_drivetrain->backRightMotor.getTemperature()) + "*C");
  pros::lcd::set_text(2, "FrontLeft: " + std::to_string(hk_drivetrain->frontLeftMotor.getTemperature()) + "*C");
  pros::lcd::set_text(3, "FrontRight: " + std::to_string(hk_drivetrain->frontRightMotor.getTemperature()) + "*C");
  pros::lcd::set_text(4, "FrontLever; Left: " +
      std::to_string(pros::c::motor_get_temperature(15))
      + "*C Right: " + std::to_string(pros::c::motor_get_temperature(16)) + "*C");
  pros::lcd::set_text(5, "BackArm; Left: " +
      std::to_string(pros::c::motor_get_temperature(12))
      + "*C Right: " + std::to_string(pros::c::motor_get_temperature(13)) + "*C");
  pros::lcd::set_text(6, "Clamp Port: " + std::to_string(pros::c::motor_get_temperature(19)) + "*C");

  hk_drivetrain->tankDrive();

  if (hk_btnFrontDown.changedToPressed()) {
    hk_frontLever->stepExtend();
  } else if (hk_btnFrontUp.changedToPressed()) {
    hk_frontLever->stepRetract();
  }

  if (hk_btnBackDown.changedToPressed()) {
    hk_backArm->stepRetract();
  } else if (hk_btnBackUp.changedToPressed()) {
    hk_backArm->stepExtend();
  }

  if (hk_btnClampToggle.changedToPressed()) {
    hk_clamp->toggle();
  }
}
