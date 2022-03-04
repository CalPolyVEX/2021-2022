#include "api.h"
#include "okapi/api.hpp"

class SteppedMotor {
private:
  std::shared_ptr<okapi::AsyncPositionController<double, double>> controller;
  std::vector<int> steps;
  int stepExtendVelocity;
  int stepRetractVelocity;
  int currentStepIndex;
  int physicalOffset;
public:
  SteppedMotor(okapi::MotorGroup motor_ports, std::initializer_list<int> steps);
  SteppedMotor(okapi::MotorGroup motor_ports, std::initializer_list<int> steps,
    int stepExtendVelocity, int stepRetractVelocity);

  void engage();
  int getCurrentStep();
  void setInitialStep(int step);

  void stepExtend();
  void stepRetract();
  void stepTo(int step, int velocity);

  void addPhysicalOffset(int offset);
  void waitUntilSettled();
};

class ToggleMotor {
private:
  SteppedMotor steppedMotor;

public:
  ToggleMotor(okapi::MotorGroup motor_ports, int enableDistance);
  void engage();
  void setInitiallyRetracted();
  void setInitiallyExtended();
  void retract();
  void extend();
  void toggle();
  void waitUntilSettled();
};
