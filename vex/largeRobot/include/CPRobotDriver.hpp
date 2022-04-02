#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "arduinoSensors.hpp"
#include "robot_specifics.h"

enum side { left, right };

class CPRobotDriver {
private:

public:
  RobotDriver(int8_t frontLeftMotorPort, int8_t frontRightMotorPort, int8_t backLeftMotorPort, int8_t backRightMotorPort);
  //Getters
  pros::Controller *getController();
  //Controller stuff
  void tankDrive();
  void arcadeDrive();
};

class CPRobotDriverBuilder {
public:
  CPRobotDriver Build();
  CPRobotDriverBuilder WithMotors(int[]);
};

class CPRobotMotor {
private:
  int port;
  pros::Motor motor;
  int dir;
  enum side side;
};
