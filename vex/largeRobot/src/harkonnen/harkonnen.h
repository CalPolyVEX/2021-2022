#ifndef HARKONNEN_H
#define HARKONNEN_H

#include "api.h"
#include "okapi/api.hpp"
#include "../steppedMotor.h"
#include "robotDriver.h"

extern RobotDriver *hk_drivetrain;
extern SteppedMotor *hk_frontLever;
extern SteppedMotor *hk_backArm;
extern ToggleMotor *hk_clamp;

void hk_opcontrol_init();
void hk_opcontrol_update();
void hk_autonomous();

#endif
