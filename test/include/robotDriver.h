#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
using namespace okapi;

class RobotDriver {
  private:
  pros::Motor frontLeftMotor;
  pros::Motor frontRightMotor;
  pros::Motor backLeftMotor;
  pros::Motor backRightMotor;
  pros::Serial arduino;
  pros::Imu gyro;

  uint64_t encoderVal = 0;

  double wheelCircumference;

  double turnPIDkP =  2.5000;
  double turnPIDkI =  0.0000;
  double turnPIDkD =  0.0000;
  double turnPIDdT = 10.0000;

  double positionPIDkP = 65.0000;
  double positionPIDkI =  0.0000;
  double positionPIDkD =  0.0000;
  double positionPIDdT = 10.0000;

  std::shared_ptr<okapi::ChassisController> chassis;

  public:
  RobotDriver(int8_t frontLeftMotorPort, int8_t frontRightMotorPort, int8_t backLeftMotorPort, int8_t backRightMotorPort, int8_t gyroPort, double wheelRad);

  void configTurnPID(double kP, double kI, double kD, double dT);
  void turnPID(double desiredTurnAngle);

  void configPositionPID(double kP, double kI, double kD, double dT);
  void positionPID(double desired_dist_inches);

  void encoderTest();
};
