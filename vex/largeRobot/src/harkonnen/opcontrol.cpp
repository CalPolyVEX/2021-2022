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
}

void hk_opcontrol_update() {
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
