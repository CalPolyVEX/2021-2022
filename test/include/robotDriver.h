#include "api.h"

class RobotDriver {
  private:
  pros::Motor frontLeftMotor;
  pros::Motor frontRightMotor;
  pros::Motor backLeftMotor;
  pros::Motor backRightMotor;
  pros::Imu gyro;
  double wheelCircumference;

  public:
  //RobotDriver(std::uint8_t frontLeftMotorPort, std::uint8_t frontRightMotorPort, std::uint8_t backLeftMotorPort, std::uint8_t backRightMotorPort, std::uint8_t gyroPort, double wheelCirc);
  RobotDriver(int frontLeftMotorPort, int frontRightMotorPort, int backLeftMotorPort, int backRightMotorPort, int gyroPort, double wheelCirc);
  void turnPID(double desiredTurnAngle);
};
