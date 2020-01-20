using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor FrontLeft;
extern motor BackLeft;
extern motor BackRight;
extern motor FrontRight;
extern motor Tilter;
extern motor IntakeL;
extern motor IntakeR;
extern motor Lift;
extern pot Pot;
extern inertial Inertial;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );