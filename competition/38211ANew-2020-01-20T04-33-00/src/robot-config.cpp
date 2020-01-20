#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
controller Controller1 = controller(primary);
motor FrontLeft = motor(PORT20, ratio18_1, true);
motor BackLeft = motor(PORT19, ratio18_1, false);
motor BackRight = motor(PORT10, ratio18_1, true);
motor FrontRight = motor(PORT9, ratio18_1, false);
motor Tilter = motor(PORT18, ratio36_1, false);
motor IntakeL = motor(PORT11, ratio18_1, false);
motor IntakeR = motor(PORT1, ratio18_1, true);
motor Lift = motor(PORT8, ratio18_1, true);
pot Pot = pot(Brain.ThreeWirePort.A);
inertial Inertial = inertial(PORT12);

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