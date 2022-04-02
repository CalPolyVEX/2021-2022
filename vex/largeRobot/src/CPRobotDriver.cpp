#include "CPRobotDriver.hpp"

class CPRobotDriver {
  CPRobotDriver() {};
};

class CPRobotBuilder {
  CPRobotBuilder() {};
  WithMotors() {};
};

class CPRobotMotor {
  CPRobotMotor(int portNum, enum side side) {
    this.port = abs(port);
    this.dir = portNum < 0 ? -1 : 1;
    this.motor = pros::Motor(this.port);
    this.side = side;
  }
};
