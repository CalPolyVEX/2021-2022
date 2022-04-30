#include "CPRobotDriver.hpp"
#include <vector>

/**
 *
 * CP ROBOT DRIVER
 *
 */
CPRobotDriver::CPRobotDriver(CPRobotMotorSet &left, CPRobotMotorSet &right, DriveMode mode) : controller(pros::E_CONTROLLER_MASTER) {
  this->leftMotorSet = &left;
  this->rightMotorSet = &right;
  this->driveMode = mode;
}

void CPRobotDriver::setSpeed(int speed) {
  if (this->leftMotorSet != NULL) {
    this->leftMotorSet->setSpeed(speed);
  }
  if (this->rightMotorSet != NULL) {
    this->rightMotorSet->setSpeed(speed);
  }
}

void CPRobotDriver::controlCycle() {
  switch(this->driveMode) {
    case Tank:
      this->leftMotorSet->setSpeed(this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y));
      this->rightMotorSet->setSpeed(this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y));
      break;
    case Arcade:
      int32_t vertical = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
      int32_t horizontal = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
      this->leftMotorSet->setSpeed((vertical + horizontal) / 127);
      this->rightMotorSet->setSpeed((vertical - horizontal) / 127);
      break;
  }
}

/**
 *
 * CP ROBOT MOTOR
 *
 */
CPRobotMotor::CPRobotMotor(int portNum) : motor(std::abs(portNum), portNum < 0) {
  this->port = std::abs(portNum);
  this->dir = portNum < 0 ? -1 : 1;
}
int CPRobotMotor::getPort() { return this->port; }
int CPRobotMotor::getDirection() { return this->dir; }
void CPRobotMotor::setSpeed(int speed) {
  this->motor.move(speed);
}

/**
 *
 * CP ROBOT MOTOR SET
 *
 */
CPRobotMotorSet::CPRobotMotorSet(std::initializer_list<int> ports) {
 for (int port : ports) {
   this->motors.push_back(CPRobotMotor(port));
 }
}
std::string CPRobotMotorSet::listMotors() {
  std::string str = "";
  for (CPRobotMotor motor : this->motors) {
    str.append(std::to_string(motor.getPort()));
    if (motor.getDirection() < 0) str.append(" (rev)");
    str.append(", ");
  }
  return str;
}

void CPRobotMotorSet::setSpeed(int speed) {
  for (CPRobotMotor motor : this->motors) {
    motor.setSpeed(speed);
  }
}
