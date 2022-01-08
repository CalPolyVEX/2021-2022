#include "api.h"

class RobotDriver {
  private:
  pros::Motor frontLeftMotor;
  pros::Motor frontRightMotor;
  pros::Motor backLeftMotor;
  pros::Motor backRightMotor;
  pros::Imu gyro;
  double wheelCircumference;

  double turnPIDkP =  2.5000;
  double turnPIDkI =  0.0000;
  double turnPIDkD =  0.0000;
  double turnPIDdT = 10.0000;

  double positionPIDkP = 65.0000;
  double positionPIDkI =  0.0000;
  double positionPIDkD =  0.0000;
  double positionPIDdT = 10.0000;

  public:
  RobotDriver(int frontLeftMotorPort, int frontRightMotorPort, int backLeftMotorPort, int backRightMotorPort, int gyroPort, double wheelCirc);

  void configTurnPID(double kP, double kI, double kD, double dT);
  void turnPID(double desiredTurnAngle);

  void configPositionPID(double kP, double kI, double kD, double dT);
  void positionPID(double desired_dist_inches);
};
