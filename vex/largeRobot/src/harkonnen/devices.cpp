#include "harkonnen.h"

#define FRONT_LOWER_VELOCITY 5000
#define FRONT_RAISE_VELOCITY 50

// Device: Drivetrain

const int8_t LEFT_WHEELS_1_PORT = -1;
const int8_t LEFT_WHEELS_2_PORT = -4;
const int8_t RIGHT_WHEELS_1_PORT = 2;
const int8_t RIGHT_WHEELS_2_PORT = 5;

const int8_t MIDDLE_WACKO_WHEEL_PORT = 9;

RobotDriver* hk_drivetrain = new RobotDriver(
  LEFT_WHEELS_1_PORT, RIGHT_WHEELS_1_PORT, LEFT_WHEELS_2_PORT, RIGHT_WHEELS_2_PORT);

// Device: Front Lever

#define FRONT_LEVER_LEFT_PORT 15
#define FRONT_LEVER_RIGHT_PORT 16

SteppedMotor* hk_frontLever  = new SteppedMotor(
  {FRONT_LEVER_LEFT_PORT, -FRONT_LEVER_RIGHT_PORT},
  {0, -500, -1000, -1450},
  FRONT_LOWER_VELOCITY, FRONT_RAISE_VELOCITY);

// Device: Back Arm

#define BACK_ARM_LEFT_PORT 12
#define BACK_ARM_RIGHT_PORT 13

SteppedMotor* hk_backArm = new SteppedMotor (
  {BACK_ARM_LEFT_PORT, -BACK_ARM_RIGHT_PORT},
  {0, -1000, -2000, -3000});

// Device: Clamp

#define CLAMP_PORT 19

ToggleMotor* hk_clamp  = new ToggleMotor({CLAMP_PORT}, 200);
