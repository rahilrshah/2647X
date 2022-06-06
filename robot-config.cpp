#include "vex.h"

using namespace vex;
using signature = vision::signature;
using code = vision::code;
// A global instance of brain used for printing to the V5 brain screen
brain Brain;

controller Controller1 = controller(primary);

motor Ldrive1 = motor(PORT11, ratio6_1, true);
motor Ldrive2 = motor(PORT12, ratio6_1, true);
motor Ldrive3_arm = motor(PORT17, ratio6_1, false);
motor Rdrive1 = motor(PORT13, ratio6_1, false);
motor Rdrive2 = motor(PORT14, ratio6_1, false);
motor Rdrive3_arm = motor(PORT16, ratio6_1,true);

motor conveyor = motor(PORT4, ratio6_1, true);
motor flippy = motor(PORT7, ratio36_1, false);
pneumatics driveShift = pneumatics(Brain.ThreeWirePort.B);
pneumatics mechShift = pneumatics(Brain.ThreeWirePort.G);
pneumatics rearMogo = pneumatics(Brain.ThreeWirePort.F);
pneumatics claw = pneumatics(Brain.ThreeWirePort.E);
pneumatics peg = pneumatics(Brain.ThreeWirePort.D);
pneumatics armAct = pneumatics(Brain.ThreeWirePort.C);

motor turret = motor(PORT15, ratio6_1, false);

inertial inert = inertial(PORT20);

triport Expander = triport(PORT18); 


rotation armRot = rotation(PORT9, true);
rotation encoder_back = rotation(PORT19, true);

// encoder encoder_back = encoder(Expander.A);
limit armBottom = limit(Brain.ThreeWirePort.A);
limit pegIn = limit(Expander.A);


line line1 = line(Brain.ThreeWirePort.H);

distance claw_range = distance(PORT1);
distance drive_range = distance(PORT6);

//rear vision is port 10
//forwrd vision is port 5



signature VIS_REAR__SIG_1 = signature (1, 0, 0, 0, 0, 0, 0, 3, 0);
signature VIS_REAR__BLU = signature (2, -3641, -2563, -3102, 9089, 11627, 10358, 5, 0);
signature VIS_REAR__RD = signature (3, 6801, 8815, 7808, -1967, -689, -1328, 3.5, 0);
signature VIS_REAR__YELLW = signature (4, 1863, 2533, 2198, -3385, -3015, -3200, 5.3, 0);
signature VIS_REAR__SIG_5 = signature (5, 0, 0, 0, 0, 0, 0, 3, 0);
signature VIS_REAR__SIG_6 = signature (6, 0, 0, 0, 0, 0, 0, 3, 0);
signature VIS_REAR__SIG_7 = signature (7, 0, 0, 0, 0, 0, 0, 3, 0);
vision VIS_REAR = vision (PORT10, 48, VIS_REAR__SIG_1, VIS_REAR__BLU, VIS_REAR__RD, VIS_REAR__YELLW, VIS_REAR__SIG_5, VIS_REAR__SIG_6, VIS_REAR__SIG_7);



signature VIS_F__YELLW = signature (1, 1429, 2325, 1877, -3917, -3443, -3680, 3.4, 0);
signature VIS_F__BLU = signature (2, -3679, -2127, -2903, 8291, 11757, 10024, 2.3, 0);
signature VIS_F__RD = signature (3, 7407, 9535, 8471, -2323, -1111, -1717, 6.9, 0);
signature VIS_F__SIG_4 = signature (4, 0, 0, 0, 0, 0, 0, 3, 0);

vision VIS_F = vision (PORT5, 23, VIS_F__YELLW, VIS_F__BLU, VIS_F__RD, VIS_F__SIG_4);
/**
 * Used to initialize code/tasks/devices added using tools in VEXcode Pro.
 *7
 * This should be called at the start of your int main function.
 */
void vexcodeInit(void) {
  // Nothing to initialize
}