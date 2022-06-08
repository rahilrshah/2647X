#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor Ldrive1 = motor(PORT20, ratio18_1, true);
motor Rdrive1 = motor(PORT19, ratio18_1, false);
motor Rdrive2 = motor(PORT17, ratio18_1, false);
motor Intake1 = motor(PORT13, ratio18_1, false);
motor Intake2 = motor(PORT1, ratio18_1, true);
motor Serial = motor(PORT11, ratio18_1, true);
motor Index = motor(PORT12, ratio6_1, true);
motor Ldrive2 = motor(PORT16, ratio18_1, true);
line topBall = line(Brain.ThreeWirePort.A);
line middleBall = line(Brain.ThreeWirePort.B);
line intakeBall = line(Brain.ThreeWirePort.C);
line middleBall2 = line(Brain.ThreeWirePort.D);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}