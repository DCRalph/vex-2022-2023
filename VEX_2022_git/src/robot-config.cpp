#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor L2 = motor(PORT19, ratio18_1, false);
motor L1 = motor(PORT20, ratio18_1, false);
motor R1 = motor(PORT11, ratio18_1, true);
motor R2 = motor(PORT12, ratio18_1, true);
motor roller = motor(PORT16, ratio18_1, false);
motor intake = motor(PORT14, ratio18_1, false);
motor cata = motor(PORT15, ratio36_1, false);
inertial Inertial = inertial(PORT16);
controller Controller1 = controller(primary);

// digital_out Launcher = digital_out(Brain.ThreeWirePort.A);
// digital_out back = digital_out(Brain.ThreeWirePort.B);

// VEXcode generated functions
// define variable for remote controller enable/disable
bool RemoteControlCodeEnabled = true;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void vexcodeInit( void ) {
  // nothing to initialize
}