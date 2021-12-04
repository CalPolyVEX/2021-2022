#include "robotDriver.h"
#define ERROR_BOUND_TURN 0.1


//RobotDriver::RobotDriver(std::uint8_t frontLeftMotorPort, std::uint8_t frontRightMotorPort, std::uint8_t backLeftMotorPort, std::uint8_t backRightMotorPort, std::uint8_t gyroPort, double wheelCirc) {
RobotDriver::RobotDriver(pros::Motor *frontLeftMotor, double wheelCirc) {
  this->frontLeftMotor = frontLeftMotor;
  /*pros::Motor frontLeftMotor(frontLeftMotorPort);
  pros::Motor frontRightMotor(frontRightMotorPort);
  pros::Motor backLeftMotor(backLeftMotorPort);
  pros::Motor backRightMotor(backRightMotorPort);

  pros::Imu gyro(gyroPort);*/

  wheelCircumference = wheelCirc;
}

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

void RobotDriver::turnPID(double desiredTurnAngle) {
  // constants for PID calculations
  const double maxSpeed = 128;
  const double minSpeed = 13;
  const int final_iterations =  8; //how many times to run within the error bound
  const double dT = 10.0000; //dT is the milliseconds between loops
  const double kP =  2.5000; //kP is the most useful part for position PID
  const double kI =  0.0000; //kI, in this case, helps ensure movement towards the end
  const double kD =  0.0000; //kD usually isn't helpful in Vex PID in general
  // initialize values to track between loops
  double error = ERROR_BOUND_TURN * 2;
  double error_prior = 0;
  double integral_prior = 0;
  double extra_iterations = final_iterations;
  // everything from here on out is measured in degrees
  double initial = gyro->get_yaw();
  while (extra_iterations > 0) {
    if (std::abs(error) < ERROR_BOUND_TURN) {
      extra_iterations -= 1;
    } else {
      extra_iterations = final_iterations;
    }
    // calculate known distances
    double actual = gyro->get_yaw() - initial;
    error = desiredTurnAngle - actual;
    //sign correct error
    while (error < -180) {
      error += 360;
    }
    while (error > 180) {
      error -= 360;
    }
    // calculate I and D
    double integral = integral_prior + (error*dT); // sum of error
    double derivative = (error - error_prior)/dT; // change in error over time
    // using PID constants, calculate output
    double output = kP * error + kI * integral + kD * derivative;
    pros::lcd::set_text(2, "initial: " + std::to_string(initial));
    pros::lcd::set_text(3, "actual: " + std::to_string(actual));
    pros::lcd::set_text(4, "desired: " + std::to_string(desiredTurnAngle));
    pros::lcd::set_text(5, "Error: " + std::to_string(error));
    pros::lcd::set_text(6, "Output: " + std::to_string(output));
    // clamp output to motor-compatible values
    output = clamp(output, maxSpeed, minSpeed);
    pros::lcd::set_text(7, "Clamped: " + std::to_string(output));
    // set motors to calculated output
    *frontLeftMotor = output;
    *backLeftMotor = output;
    *frontRightMotor = output;
    *backRightMotor = output;
    // record new prior values
    error_prior = error;
    integral_prior = integral;
    // delay by dT
    pros::delay(dT);
  }
}
