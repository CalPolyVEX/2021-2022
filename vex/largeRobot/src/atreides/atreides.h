#ifndef ATREIDES_H
#define ATREIDES_H

#include "api.h"
#include "okapi/api.hpp"
#include "../steppedMotor.h"
#include "robotDriver.h"

extern RobotDriver* at_drivetrain;
extern pros::Motor at_frontIntake;
extern pros::Motor Arm1;
extern pros::Motor Arm2;
//extern SteppedMotor* at_backClawArm;
extern SteppedMotor at_clawDeployMotor;
extern bool at_clawClosed;
extern pros::ADIDigitalOut at_clawClosePiston;

void at_opcontrol_init();
void at_opcontrol_update();
void at_autonomous();

#endif
