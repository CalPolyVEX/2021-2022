#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor leftMotorA = motor(PORT2, ratio18_1, false);
motor leftMotorB = motor(PORT3, ratio18_1, false);
motor_group LeftDriveSmart = motor_group(leftMotorA, leftMotorB);
motor rightMotorA = motor(PORT17, ratio18_1, true);
motor rightMotorB = motor(PORT15, ratio18_1, true);
motor_group RightDriveSmart = motor_group(rightMotorA, rightMotorB);
drivetrain Drivetrain = drivetrain(LeftDriveSmart, RightDriveSmart, 319.19, 295, 40, mm, 1);
motor Motor4 = motor(PORT4, ratio18_1, false);
motor MotorGroup5MotorA = motor(PORT5, ratio18_1, false);
motor MotorGroup5MotorB = motor(PORT6, ratio18_1, true);
motor_group MotorGroup5 = motor_group(MotorGroup5MotorA, MotorGroup5MotorB);
motor Motor16 = motor(PORT16, ratio18_1, false);
digital_out DigitalOutA = digital_out(Brain.ThreeWirePort.A);
controller Controller1 = controller(primary);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;
// define variables used for controlling motors based on controller inputs
bool Controller1LeftShoulderControlMotorsStopped = true;
bool Controller1RightShoulderControlMotorsStopped = true;
bool DrivetrainLNeedsToBeStopped_Controller1 = true;
bool DrivetrainRNeedsToBeStopped_Controller1 = true;
bool Controller1UpDownButtonsControlMotorsStopped = true;

// define a task that will handle monitoring inputs from Controller1
int rc_auto_loop_function_Controller1() {
  // process the controller input every 20 milliseconds
  // update the motors based on the input values
  while(true) {
    if(RemoteControlCodeEnabled) {
      // calculate the drivetrain motor velocities from the controller joystick axies
      // left = Axis3 + Axis1
      // right = Axis3 - Axis1
      int drivetrainLeftSideSpeed = Controller1.Axis3.position() + Controller1.Axis1.position();
      int drivetrainRightSideSpeed = Controller1.Axis3.position() - Controller1.Axis1.position();
      
      // check if the value is inside of the deadband range
      if (drivetrainLeftSideSpeed < 5 && drivetrainLeftSideSpeed > -5) {
        // check if the left motor has already been stopped
        if (DrivetrainLNeedsToBeStopped_Controller1) {
          // stop the left drive motor
          LeftDriveSmart.stop();
          // tell the code that the left motor has been stopped
          DrivetrainLNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the left motor nexttime the input is in the deadband range
        DrivetrainLNeedsToBeStopped_Controller1 = true;
      }
      // check if the value is inside of the deadband range
      if (drivetrainRightSideSpeed < 5 && drivetrainRightSideSpeed > -5) {
        // check if the right motor has already been stopped
        if (DrivetrainRNeedsToBeStopped_Controller1) {
          // stop the right drive motor
          RightDriveSmart.stop();
          // tell the code that the right motor has been stopped
          DrivetrainRNeedsToBeStopped_Controller1 = false;
        }
      } else {
        // reset the toggle so that the deadband code knows to stop the right motor next time the input is in the deadband range
        DrivetrainRNeedsToBeStopped_Controller1 = true;
      }
      
      // only tell the left drive motor to spin if the values are not in the deadband range
      if (DrivetrainLNeedsToBeStopped_Controller1) {
        RightDriveSmart.setVelocity((3*drivetrainLeftSideSpeed), percent);
        RightDriveSmart.spin(forward);
      }
      // only tell the right drive motor to spin if the values are not in the deadband range
      if (DrivetrainRNeedsToBeStopped_Controller1) {
        LeftDriveSmart.setVelocity((3*drivetrainRightSideSpeed), percent);
        LeftDriveSmart.spin(forward);
      }
      // check the ButtonL1/ButtonL2 status to control MotorGroup5
      if (Controller1.ButtonL1.pressing()) {
        MotorGroup5.setVelocity(50,velocityUnits::pct);
        MotorGroup5.spin(reverse);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonL2.pressing()) {
        MotorGroup5.setVelocity(50,velocityUnits::pct);
        MotorGroup5.spin(forward);
        Controller1LeftShoulderControlMotorsStopped = false;
      } else if (!Controller1LeftShoulderControlMotorsStopped) {
        MotorGroup5.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1LeftShoulderControlMotorsStopped = true;
      }
      // check the ButtonR1/ButtonR2 status to control Motor4
      if (Controller1.ButtonR1.pressing()) {
        Motor4.setVelocity(80,velocityUnits::pct);
        Motor4.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (Controller1.ButtonR2.pressing()) {
        Motor4.setVelocity(80,velocityUnits::pct);
        Motor4.spin(forward);
        Controller1RightShoulderControlMotorsStopped = false;
      } else if (!Controller1RightShoulderControlMotorsStopped) {
        Motor4.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1RightShoulderControlMotorsStopped = true;
      }
      if (Controller1.ButtonUp.pressing()) {
        Motor16.setVelocity(50,velocityUnits::pct);
        Motor16.spin(reverse);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (Controller1.ButtonDown.pressing()) {
        Motor16.setVelocity(50,velocityUnits::pct);
        Motor16.spin(forward);
        Controller1UpDownButtonsControlMotorsStopped = false;
      } else if (!Controller1UpDownButtonsControlMotorsStopped) {
        Motor16.stop();
        // set the toggle so that we don't constantly tell the motor to stop when the buttons are released
        Controller1UpDownButtonsControlMotorsStopped = true;
      }
      if (Controller1.ButtonX.pressing()){
        DigitalOutA.set(true);
      }
      else if (Controller1.ButtonB.pressing()){
        DigitalOutA.set(false);
        }
    }
    // wait before repeating the process
    wait(20, msec);
  }
  return 0;
  }


/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  task rc_auto_loop_task_Controller1(rc_auto_loop_function_Controller1);
}