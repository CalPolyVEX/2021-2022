// include
#include "robotDriver.h"
// define
#define ERROR_BOUND_DRIVE 0.001
#define ERROR_BOUND_TURN 0.1
// class
RobotDriver::RobotDriver(int8_t frontLeftMotorPort, int8_t frontRightMotorPort, int8_t backLeftMotorPort, int8_t backRightMotorPort, int8_t gyroPort, double wheelRad)
 : frontLeftMotor(frontLeftMotorPort), frontRightMotor(frontRightMotorPort), backLeftMotor(backLeftMotorPort), backRightMotor(backRightMotorPort),
 gyro(gyroPort), a('A'), b('B'), i('C')
{
  int a = 4;
  chassis = okapi::ChassisControllerBuilder()
  .withMotors({frontLeftMotorPort, backLeftMotorPort}, {(int8_t)-frontRightMotorPort, (int8_t)-backRightMotorPort})
  .withDimensions(AbstractMotor::gearset::green, {{4.1_in, 14.6_in}, imev5GreenTPR})
  .build();

  wheelCircumference = wheelRad * M_PI;

  turnPIDdT = 10.0000;
  turnPIDkP =  2.5000;
  turnPIDkI =  0.0000;
  turnPIDkD =  0.0000;
}
// utility functions
double clamp(double val, double max, double min) {
 double sign = 1;
 if (val < 0) {
   sign = -1;
 }
 if (std::abs(val) < min) {
   return min * sign;
 } else if (std::abs(val) > max) {
   return max * sign;
 } else {
   return val;
 }
}
// getter/setter methods
void RobotDriver::configTurnPID(double kP, double kI, double kD, double dT) {
  this->turnPIDdT = dT;
  this->turnPIDkP = kP;
  this->turnPIDkI = kI;
  this->turnPIDkD = kD;
}
void RobotDriver::configPositionPID(double kP, double kI, double kD, double dT) {
  this->positionPIDdT = dT;
  this->positionPIDkP = kP;
  this->positionPIDkI = kI;
  this->positionPIDkD = kD;
}
// driving functions
void RobotDriver::turnPID(double desiredTurnAngle) {
  // constants for PID calculations
  const double maxSpeed = 128;
  const double minSpeed = 13;
  const int final_iterations =  8; //how many times to run within the error bound
  // initialize values to track between loops
  double error = ERROR_BOUND_TURN * 2;
  double error_prior = 0;
  double integral_prior = 0;
  double extra_iterations = final_iterations;
  // everything from here on out is measured in degrees
  double initial = gyro.get_yaw();
  while (extra_iterations > 0) {
    if (std::abs(error) < ERROR_BOUND_TURN) {
      extra_iterations -= 1;
    } else {
      extra_iterations = final_iterations;
    }
    // calculate known distances
    double actual = gyro.get_yaw() - initial;
    error = desiredTurnAngle - actual;
    //sign correct error
    while (error < -180) {
      error += 360;
    }
    while (error > 180) {
      error -= 360;
    }
    // calculate I and D
    double integral = integral_prior + (error*this->turnPIDdT); // sum of error
    double derivative = (error - error_prior)/this->turnPIDdT; // change in error over time
    // using PID constants, calculate output
    double output = this->turnPIDkP * error + this->turnPIDkI * integral + this->turnPIDkD * derivative;
    pros::lcd::set_text(2, "initial: " + std::to_string(initial));
    pros::lcd::set_text(3, "actual: " + std::to_string(actual));
    pros::lcd::set_text(4, "desired: " + std::to_string(desiredTurnAngle));
    pros::lcd::set_text(5, "Error: " + std::to_string(error));
    pros::lcd::set_text(6, "Output: " + std::to_string(output));
    // clamp output to motor-compatible values
    output = clamp(output, maxSpeed, minSpeed);
    pros::lcd::set_text(7, "Clamped: " + std::to_string(output));
    // set motors to calculated output
    frontLeftMotor = output;
    backLeftMotor = output;
    frontRightMotor = output;
    backRightMotor = output;
    // record new prior values
    error_prior = error;
    integral_prior = integral;
    // delay by dT
    pros::delay(this->turnPIDdT);
  }
}
void RobotDriver::positionPID(double desired_dist_inches) {
	 // constants for PID calculations
	 const double maxSpeed = 128;
	 // initialize values to track between loops
	 double error = ERROR_BOUND_DRIVE * 2;
	 double error_prior = 0;
	 double integral_prior = 0;
	 // everything from here on out is measured in revolutions
	 double desired = desired_dist_inches / this->wheelCircumference;
	 this->frontLeftMotor.set_encoder_units(pros::E_MOTOR_ENCODER_ROTATIONS);
	 double initialMotorPosition = this->frontLeftMotor.get_position();
	 while (abs((int)error) > ERROR_BOUND_DRIVE) {
		 // calculate known distances
		 double actual = (this->frontLeftMotor.get_position()) - initialMotorPosition;
		 error = desired - actual;
		 // calculate I and D
		 double integral = integral_prior + (error*this->positionPIDdT); // sum of error
		 double derivative = (error - error_prior)/this->positionPIDdT; // change in error over time
		 // using PID constants, calculate output
		 double output = this->positionPIDkP * error + this->positionPIDkI * integral + this->positionPIDkD * derivative;
		 pros::lcd::set_text(2, "integral: " + std::to_string(integral));
		 pros::lcd::set_text(3, "derivative: " + std::to_string(derivative));
		 pros::lcd::set_text(4, "Error: " + std::to_string(error));
		 pros::lcd::set_text(5, "Output: " + std::to_string(output));
		 // clamp output to motor-compatible values
		 output = fmin(fmax(output, -maxSpeed), maxSpeed);
		 pros::lcd::set_text(6, "Clamped: " + std::to_string(output));
		 // set motors to calculated output
		 this->frontLeftMotor = output;
		 this->backLeftMotor = output;
		 this->frontRightMotor = -output;
		 this->backRightMotor = -output;
		 // record new prior values
		 error_prior = error;
		 integral_prior = integral;
		 // delay by dT
		 pros::delay(this->positionPIDdT);
	 }
}

void RobotDriver::encoderTest() {
    aVal = a.get_value();
    bVal = b.get_value();
    iVal = i.get_value();
    pros::lcd::set_text(1, "A: " + std::to_string(aVal));
    pros::lcd::set_text(2, "B: " + std::to_string(bVal));
    pros::lcd::set_text(3, "I: " + std::to_string(iVal));
}
