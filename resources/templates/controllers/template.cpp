// File:          template.cpp
// Date:
// Description:
// Author:
// Modifications:

// You may need to add include files such as
// <omnisim/DistanceSensor.hpp>, <omnisim/Motor.hpp>, etc. <omnisim/...>
// is the only include path: the legacy <webots/...> forwarders were
// removed, so port any older controller by rewriting webots/ to
// omnisim/ in its includes.
#include <omnisim/Robot.hpp>

// All the controller classes are declared in the C++ namespace `omnisim`.
// The namespace is part of the compiled C++ ABI (it is baked into every
// mangled symbol in libCppController), so rebuild any controller you port.
using namespace omnisim;

// This is the main program of your controller.
// It creates an instance of your Robot instance, launches its
// function(s) and destroys it at the end of the execution.
// Note that only one instance of Robot should be created in
// a controller program.
// The arguments of the main function can be specified by the
// "controllerArgs" field of the Robot node
int main(int argc, char **argv) {
  // create the Robot instance.
  Robot *robot = new Robot();

  // get the time step of the current world.
  int timeStep = (int)robot->getBasicTimeStep();

  // You should insert a getDevice-like function in order to get the
  // instance of a device of the robot. Something like:
  //  Motor *motor = robot->getMotor("motorname");
  //  DistanceSensor *ds = robot->getDistanceSensor("dsname");
  //  ds->enable(timeStep);

  // Main loop:
  // - perform simulation steps until OmniSim is stopping the controller
  while (robot->step(timeStep) != -1) {
    // Read the sensors:
    // Enter here functions to read sensor data, like:
    //  double val = ds->getValue();

    // Process sensor data here.

    // Enter here functions to send actuator commands, like:
    //  motor->setPosition(10.0);
  };

  // Enter here exit cleanup code.

  delete robot;
  return 0;
}
