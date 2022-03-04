#include "steppedMotor.h"

SteppedMotor::SteppedMotor(okapi::MotorGroup motor_ports, std::initializer_list<int> steps) {
  this->controller = okapi::AsyncPosControllerBuilder().withMotor(motor_ports).build();
  this->steps = std::vector(steps);
  this->stepExtendVelocity = 5000;
  this->stepRetractVelocity = 5000;
  this->currentStepIndex = 0;
  this->physicalOffset = 0;
}

SteppedMotor::SteppedMotor(okapi::MotorGroup motor_ports, std::initializer_list<int> steps,
  int stepExtendVelocity, int stepRetractVelocity) {
  this->controller = okapi::AsyncPosControllerBuilder().withMotor(motor_ports).build();
  this->steps = std::vector(steps);
  this->stepExtendVelocity = stepExtendVelocity;
  this->stepRetractVelocity = stepRetractVelocity;
  this->currentStepIndex = 0;
  this->physicalOffset = 0;
}

void SteppedMotor::engage() {
  this->controller->setTarget(this->steps.at(this->currentStepIndex) + this->physicalOffset);
}

int SteppedMotor::getCurrentStep() {
  return this->currentStepIndex;
}

void SteppedMotor::setInitialStep(int step) {
  this->currentStepIndex = step;
}

void SteppedMotor::stepExtend() {
  if (this->currentStepIndex >= this->steps.size() - 1) {
    return;
  }

  this->currentStepIndex++;
  this->controller->setMaxVelocity(this->stepExtendVelocity);

  engage();
}

void SteppedMotor::stepRetract() {
  if (this->currentStepIndex <= 0) {
    return;
  }

  this->currentStepIndex--;
  this->controller->setMaxVelocity(this->stepRetractVelocity);

  engage();
}

void SteppedMotor::stepTo(int step, int velocity) {
  if (step < 0 || step >= this->steps.size()) {
    throw std::invalid_argument( "out of bounds step" );
  }

  this->currentStepIndex = step;
  this->controller->setMaxVelocity(velocity);

  engage();
}

void SteppedMotor::addPhysicalOffset(int offset) {
  this->physicalOffset += offset;

  engage();
}

void SteppedMotor::waitUntilSettled() {
  this->controller->waitUntilSettled();
}

ToggleMotor::ToggleMotor(okapi::MotorGroup motor_ports, int enableDistance)
  : steppedMotor(SteppedMotor(motor_ports, {0, enableDistance})) {
    // just initializing stepped motor
}

void ToggleMotor::engage() {
  steppedMotor.engage();
}

void ToggleMotor::setInitiallyRetracted() {
  steppedMotor.setInitialStep(0);
}

void ToggleMotor::setInitiallyExtended() {
  steppedMotor.setInitialStep(1);
}

void ToggleMotor::retract() {
  steppedMotor.stepRetract();
}

void ToggleMotor::extend() {
  steppedMotor.stepExtend();
}

void ToggleMotor::toggle() {
  if (steppedMotor.getCurrentStep() == 1) {
    retract();
  } else {
    extend();
  }
}

void ToggleMotor::waitUntilSettled() {
  steppedMotor.waitUntilSettled();
}
