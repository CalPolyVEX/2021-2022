#include "api.h"
#include "okapi/api.hpp"
#include "okapi/impl/device/motor/motorGroup.hpp"
using namespace okapi;

// typedef struct ArmButtonPorts {
//   int port1;
//   int port2;
//   int upButton;
//   int downButton;
// } ArmButtonPorts;

class RobotDriver {
  private:
  pros::Motor frontLeftMotor;
  pros::Motor frontRightMotor;
  pros::Motor backLeftMotor;
  pros::Motor backRightMotor;
  pros::Imu gyro;
  pros::Controller controller;

  int numEncoders;
  std::vector<int> encoderVals;
  int encoderPPR;

  // std::vector<ArmButtonPorts> armButtonPorts;

  double wheelCircumference;

  double turnPIDkP =  2.5000;
  double turnPIDkI =  0.0000;
  double turnPIDkD =  0.0000;
  double turnPIDdT = 10.0000;
  int turnPIDMinSpeed = 15;
  int turnPIDMaxSpeed = 128;

  double positionPIDkP = 65.0000;
  double positionPIDkI =  0.0000;
  double positionPIDkD =  0.0000;
  double positionPIDdT = 10.0000;

  std::shared_ptr<okapi::ChassisController> chassis;

  public:
  RobotDriver(int8_t frontLeftMotorPort, int8_t frontRightMotorPort, int8_t backLeftMotorPort, int8_t backRightMotorPort, int8_t gyroPort, double wheelRad);
  //Getters
  pros::Controller *getController();
  //PID stuff
  void configTurnPID(double kP, double kI, double kD, double dT, int min, int max);
  void turnPID(double desiredTurnAngle);
  void turnPIDAndRecalibrate(double desiredTurnAngle);
  void recalibrateGyro();
  void configPositionPID(double kP, double kI, double kD, double dT);
  void positionPID(double desired_dist_inches);
  //Encoders
  void configEncoders(int numE, int ppr); //tells the driver how many encoders
  void updateEncoderVals(); //updates encoder vals
  int32_t getEncoderVal(int index); //just reads encoder vals, without checking for new data
  int32_t readEncoder(int index); //updates encoder vals, and returns the specified val
  //Controller stuff
  void tankDrive();
  void arcadeDrive();
  // void addArmButton(int port1, int port2, int upButton, int downButton);
  // void armButtons();
};
