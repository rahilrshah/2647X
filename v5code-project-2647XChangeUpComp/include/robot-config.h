using namespace vex;

extern brain Brain;

// VEXcode devices
extern controller Controller1;
extern motor Ldrive1;
extern motor Rdrive1;
extern motor Rdrive2;
extern motor Intake1;
extern motor Intake2;
extern motor Serial;
extern motor Index;
extern motor Ldrive2;
extern line topBall;
extern line middleBall;
extern line intakeBall;
extern line middleBall2;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Text.
 * 
 * This should be called at the start of your int main function.
 */
void  vexcodeInit( void );