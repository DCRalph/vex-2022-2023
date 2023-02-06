#include "vex.h"

using namespace vex;

brain Brain;

// VEXcode device constructors
motor L2 = motor(PORT19, ratio18_1, false);
motor L1 = motor(PORT20, ratio18_1, false);
motor R1 = motor(PORT11, ratio18_1, true);
motor R2 = motor(PORT12, ratio18_1, true);
motor roller = motor(PORT16, ratio18_1, false);
motor intake = motor(PORT14, ratio18_1, false);
motor cata = motor(PORT15, ratio36_1, false);
inertial Inertial = inertial(PORT17);
controller Controller1 = controller(primary);

digital_out Launcher = digital_out(Brain.ThreeWirePort.A);

motor_group leftGroup(L1, L2);
motor_group rightGroup(R1, R2);

drivetrain driveTrain(leftGroup, rightGroup, 320, 382, 334, mm, 1.5);

// digital_out back = digital_out(Brain.ThreeWirePort.B);
