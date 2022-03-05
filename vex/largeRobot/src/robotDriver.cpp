// include
#include "robotDriver.h"

#ifdef LARGE_ROBOT_HARKONNEN
#include "harkonnen/harkonnen.h"
#endif
// define

// class
RobotDriver::RobotDriver(int8_t frontLeftMotorPort, int8_t frontRightMotorPort, int8_t backLeftMotorPort, int8_t backRightMotorPort)
#ifdef LARGE_ROBOT_HARKONNEN
/*
const int8_t LEFT_WHEELS_1_PORT = -1;
const int8_t LEFT_WHEELS_2_PORT = -4;
const int8_t RIGHT_WHEELS_1_PORT = 2;
const int8_t RIGHT_WHEELS_2_PORT = 5;
*/

 : frontLeftMotor(-1),
   frontRightMotor(2),
   backLeftMotor(-4),
   backRightMotor(5),
#else
: frontLeftMotor(frontLeftMotorPort),
  frontRightMotor(frontRightMotorPort),
  backLeftMotor(backLeftMotorPort),
  backRightMotor(backRightMotorPort),
#endif
 controller(pros::E_CONTROLLER_MASTER)
{
  //use okapi chasis for drive PID
  chassis = okapi::ChassisControllerBuilder()
  .withMotors({frontLeftMotorPort, backLeftMotor}, {frontRightMotor, backRightMotor})
#ifndef USE_INTEGRATED_ENCODERS
#ifndef ARDUINO_MIDDLE_ENCODER
  .withSensors(std::make_shared<ArduinoEncoder>(arduino_encoder_create(ARDUINO_LEFT_ENCODER)),
    std::make_shared<ArduinoEncoder>(arduino_encoder_create(ARDUINO_RIGHT_ENCODER)))
#else
  .withSensors(std::make_shared<ArduinoEncoder>(arduino_encoder_create(ARDUINO_LEFT_ENCODER)),
    std::make_shared<ArduinoEncoder>(arduino_encoder_create(ARDUINO_RIGHT_ENCODER)),
    std::make_shared<ArduinoEncoder>(arduino_encoder_create(ARDUINO_MIDDLE_ENCODER)))
#endif
#endif
  .withDimensions(AbstractMotor::gearset::green, {{WHEEL_DIAMETER, WHEEL_TRACK,
    LENGTH_TO_MIDDLE_WHEEL, MIDDLE_IDLE_WHEEL_DIAMETER}, imev5GreenTPR})
  .build();
}

void RobotDriver::tankDrive() {
  int left = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int right = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_Y);

  #ifdef LARGE_ROBOT_HARKONNEN
  pros::c::motor_move(1, -left);
  pros::c::motor_move(4, -left);
  pros::c::motor_move(2, right);
  pros::c::motor_move(5, right);
  pros::c::motor_move(MIDDLE_WACKO_WHEEL_PORT, -(left + right) / 2);
  #else
  this->frontLeftMotor = left;
  this->backLeftMotor = left;
  this->frontRightMotor = right;
  this->backRightMotor = right;
  #endif
}

void RobotDriver::arcadeDrive() {
  int32_t vertical = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_LEFT_Y);
  int32_t horizontal = this->controller.get_analog(pros::E_CONTROLLER_ANALOG_RIGHT_X);
  this->frontLeftMotor.moveVoltage((vertical + horizontal) * 12000 / 127);
  this->backLeftMotor.moveVoltage((vertical + horizontal) * 12000 / 127);
  this->frontRightMotor.moveVoltage((vertical - horizontal) * 12000 / 127);
  this->backRightMotor.moveVoltage((vertical - horizontal) * 12000 / 127);
}

pros::Controller *RobotDriver::getController() {
  return &(this->controller);
}
