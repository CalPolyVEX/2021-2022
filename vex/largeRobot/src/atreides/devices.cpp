#include "atreides.h"

// Device: Drivetrain

const int8_t LEFT_WHEELS_1_PORT = -2;
const int8_t LEFT_WHEELS_2_PORT = -3;
const int8_t RIGHT_WHEELS_1_PORT = 17;
// moved to port 15 since port 18 was not working quite right
// the wire kept on falling about
const int8_t RIGHT_WHEELS_2_PORT = 15;

RobotDriver* at_drivetrain = new RobotDriver(
  LEFT_WHEELS_1_PORT, RIGHT_WHEELS_1_PORT, LEFT_WHEELS_2_PORT, RIGHT_WHEELS_2_PORT);

CPRobotBuilder = new CPRobotDriverBuilder();
CPRobotDriver testDriver = CPRobotDriverBuilder.WithMotors()

// Device: Front Intake

pros::Motor at_frontIntake(4);

// Device: Back Claw Arm

#define BACK_ARM_LEFT_PORT 5
#define BACK_ARM_RIGHT_PORT 6

pros::Motor Arm1(5);
pros::Motor Arm2(6);

// pros::Motor at_backArm1(5);
// pros::Motor at_backArm2(6);

SteppedMotor* at_backClawArm = new SteppedMotor (
  {BACK_ARM_LEFT_PORT, -BACK_ARM_RIGHT_PORT},
  {0, -2000, -4000, -8000},
  75, 75);

// Device: Claw Deploy Motor

SteppedMotor at_clawDeployMotor(
  {16},
  {0, -500, -2500}); //greater negative number for outward position

// Device: Claw Close Piston

bool at_clawClosed = false;
pros::ADIDigitalOut at_clawClosePiston ('A');
