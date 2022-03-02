#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
#include "arduinoSensors.hpp"
#include "robot_specifics.h"
using namespace okapi;

class RobotDriver {
  private:
  pros::Motor frontLeftMotor;
  pros::Motor frontRightMotor;
  pros::Motor backLeftMotor;
  pros::Motor backRightMotor;
  pros::Controller controller;

public:
  std::shared_ptr<okapi::ChassisController> chassis;

  RobotDriver(int8_t frontLeftMotorPort, int8_t frontRightMotorPort, int8_t backLeftMotorPort, int8_t backRightMotorPort);
  //Getters
  pros::Controller *getController();
  //Controller stuff
  void tankDrive();
  void arcadeDrive();
};
