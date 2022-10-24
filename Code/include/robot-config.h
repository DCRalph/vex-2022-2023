using namespace vex;

extern brain Brain;

// VEXcode devices
extern motor L3;
extern motor L2;
extern motor R3;
extern motor R2;
extern controller Controller1;
extern inertial Inertial;
extern motor L1;
extern motor R1;
extern motor roller;
extern digital_out Launcher;
extern digital_out back;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );