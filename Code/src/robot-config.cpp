#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;

// A global instance of brain used for printing to the V5 Brain screen
brain  Brain;

// VEXcode device constructors
motor L3 = motor(PORT20, ratio18_1, false);
motor L2 = motor(PORT19, ratio18_1, true);
motor R3 = motor(PORT11, ratio18_1, true);
motor R2 = motor(PORT12, ratio18_1, false);
controller Controller1 = controller(primary);
inertial Inertial = inertial(PORT16);
motor L1 = motor(PORT18, ratio18_1, false);
motor R1 = motor(PORT13, ratio18_1, true);
motor roller = motor(PORT14, ratio18_1, false);
digital_out Launcher = digital_out(Brain.ThreeWirePort.A);
digital_out back = digital_out(Brain.ThreeWirePort.B);

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