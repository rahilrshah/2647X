using namespace vex;

extern brain Brain;

/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *
 * This should be called at the start of your int main function.
 */
extern controller Controller1;
extern motor Ldrive1;
extern motor Ldrive2;
extern motor Ldrive3_arm;
extern motor Rdrive1;
extern motor Rdrive2;
extern motor Rdrive3_arm;
extern motor conveyor;
extern motor flippy;
extern pneumatics driveShift;
extern pneumatics mechShift;
extern pneumatics rearMogo;
extern pneumatics claw;
extern pneumatics peg;
extern pneumatics armAct;
extern rotation armRot;
extern triport Expander;
extern inertial inert;
extern rotation encoder_back;
extern limit armBottom;
extern limit pegIn;
extern motor turret;

extern line line1;

using signature = vision::signature;

extern signature VIS_REAR__SIG_1;
extern signature VIS_REAR__BLU;
extern signature VIS_REAR__RD;
extern signature VIS_REAR__YELLW;
extern signature VIS_REAR__SIG_5;
extern signature VIS_REAR__SIG_6;
extern signature VIS_REAR__SIG_7;
extern vision VIS_REAR;

extern signature VIS_F__YELLW;
extern signature VIS_F__BLU;
extern signature VIS_F__RD;
extern signature VIS_F__SIG_4;
extern signature VIS_F__SIG_5;
extern signature VIS_F__SIG_6;
extern signature VIS_F__SIG_7;
extern vision VIS_F;
extern distance claw_range;
extern distance drive_range;

void vexcodeInit(void);
