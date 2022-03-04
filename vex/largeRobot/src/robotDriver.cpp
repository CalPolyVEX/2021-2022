// include
#include "robotDriver.h"
// define

// class
RobotDriver::RobotDriver(int8_t frontLeftMotorPort, int8_t frontRightMotorPort, int8_t backLeftMotorPort, int8_t backRightMotorPort)
 : frontLeftMotor(frontLeftMotorPort),
   frontRightMotor(frontRightMotorPort),
   backLeftMotor(backLeftMotorPort),
   backRightMotor(backRightMotorPort),
 controller(pros::E_CONTROLLER_MASTER)
{
  //use okapi chasis for drive PID
  chassis = okapi::ChassisControllerBuilder()
  .withMotors({-3, -2}, {15, 17})
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
  this->frontLeftMotor = left;
  this->backLeftMotor = left;
  this->frontRightMotor = right;
  this->backRightMotor = right;
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
