#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include <vector>
// #include "arduinoSensors.hpp"

enum DriveMode { Tank, Arcade };

class CPRobotMotor {
private:
  int port;
  pros::Motor motor;
  int dir;
public:
  CPRobotMotor(int portNum);
  int getPort();
  int getDirection();
  void setSpeed(int speed);
};

class CPRobotMotorSet {
private:
  std::vector<CPRobotMotor> motors;
public:
  CPRobotMotorSet(std::initializer_list<int> ports);
  std::string listMotors();
  void setSpeed(int speed);
};

class CPRobotDriver {
private:
  CPRobotMotorSet *leftMotorSet;
  CPRobotMotorSet *rightMotorSet;
  DriveMode driveMode;
  pros::Controller controller;
public:
  CPRobotDriver(CPRobotMotorSet &left, CPRobotMotorSet &right, DriveMode mode);
  void setSpeed(int speed);
  void controlCycle();
};

// class CPRobotDriverBuilder {
// public:
//   CPRobotDriver Build();
//   CPRobotDriverBuilder WithMotors(int[]);
// };
