/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// Robot Configuration:
// [Name]               [Type]        [Port(s)]
// Controller1          controller                    
// Ldrive1              motor         20              
// Rdrive1              motor         19              
// Rdrive2              motor         17              
// Intake1              motor         13              
// Intake2              motor         1               
// Serial               motor         11              
// Index                motor         12              
// Ldrive2              motor         16              
// topBall              line          A               
// middleBall           line          B               
// intakeBall           line          C               
// middleBall2          line          D               
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here


void reset(){
  Rdrive1.resetRotation();
  Rdrive1.resetPosition();
  Rdrive2.resetRotation();
  Rdrive2.resetPosition();
  Ldrive1.resetRotation();
  Ldrive1.resetPosition();
  Ldrive2.resetRotation();
  Ldrive2.resetPosition();
  Intake1.resetRotation();
  Intake1.resetPosition();
  Intake2.resetRotation();
  Intake2.resetPosition();
  Serial.resetRotation();
  Serial.resetPosition();
  Index.resetRotation();
  Index.resetPosition();
}



// // fwd constants
// double LkP2 = 1.5;
// double LkP1 = 1.5;
// double RkP1 = 1.5;
// double RkP2 = 1.5;

// //fwd variables
// int leftError1; //current - desired
// int rightError1;
// int leftError2;
// int rightError2;
// double leftPower1;
// double rightPower1;
// double leftPower2;
// double rightPower2;

// void RPID1(int RSpeed1){
//   int RValue1 = RSpeed1;
//   if((Rdrive1.velocity(percent) < RValue1-5 || Rdrive1.velocity(percent) > RValue1+5)){
//     int Rposition1 = (Rdrive1.velocity(percent));
    
//     //P
//     rightError1 = RValue1 - Rposition1;

//     rightPower1 = (rightError1*RkP1);
//     }

//     Rdrive1.spin(forward, rightPower1, percent);

//     vex::task::sleep(20);
// }

// void LPID1(int LSpeed1){
//   int LValue1 = LSpeed1;
//   if((Ldrive1.velocity(percent) < LValue1-5 || Ldrive1.velocity(percent) > LValue1+5)){
//     int Lposition1 = (Ldrive1.velocity(percent));
    
//     //P
//     leftError1 = LValue1 - Lposition1;

//     leftPower1 = (leftError1*LkP1);
//     }

//     Ldrive1.spin(forward, leftPower1, percent);

//     vex::task::sleep(20);
// }

// void LPID2(int LSpeed2){
//   int LValue2 = LSpeed2;
//   if((Ldrive2.velocity(percent) < LValue2-5 || Ldrive2.velocity(percent) > LValue2+5)){
//     int Lposition2 = (Ldrive2.velocity(percent));
    
//     //P
//     leftError2 = LValue2 - Lposition2;

//     leftPower2 = (leftError2*LkP2);
//     }

//     Ldrive2.spin(forward, leftPower2, percent);

//     vex::task::sleep(20);
// }

// void RPID2(int RSpeed2){
//   int RValue2 = RSpeed2;
//   if((Rdrive2.velocity(percent) < RValue2-5 || Rdrive2.velocity(percent) > RValue2+5)){
//     int Rposition2 = (Rdrive2.velocity(percent));
    
//     //P
//     rightError2 = RValue2 - Rposition2;

//     rightPower2 = (rightError2*RkP2);
//     }

//     Rdrive2.spin(forward, rightPower2, percent);

//     vex::task::sleep(20);
// }

/*
void LbackControl(){
  int axis2 = Controller1.Azis2.position();
  int 
   int Lback2 = -Controller1.Axis2.position()-Controller1.Axis3.position() + (Controller1.Axis1.position()^3/4) + Controller1.Axis4.position();
   int Lback = Lback2*100;
   int LbackSpeed2 = Lback2;
   if(Lback > LbackPrev+10){
     LbackSpeed2 = LbackPrev + 4;
   }
   else if(Lback < LbackSpeed-10){
     LbackSpeed2 = LbackSpeed-4;
   }
   else{
     
   }
}

*/

//sets the booleans used to determine if a ball is in a slot
bool slot1; 
bool slot2;
bool slot2space;
bool slot3;
float shootSpeed; //used to oscillate the indexer when shooting

//this function just dictates how the indexer will run depending on the circumstances
void slotMove(int slot){
  if(slot == 1){ //if the input into the function is 1, this means that the ball will move to slot 1.
    if(slot1 == false){ //this is a simple if-then loop which says that if slot 1 is empty,
      Index.spin(forward,600,rpm); //and the command is given to mave a ball, it will be moved to slot 1.
    }
    else{
      Index.stop();
    }
  }
  if(slot == 2){ //this says that if the input is 2,
    if(slot1 == true){ // then if slot 1 is full 
      if(slot2 == false){ //but slot 2 is empty,
        Index.spin(forward,-600,rpm); //the Indexer will move a ball down from slot 1 to slot 2
      }
      else{
      Index.stop();
      }
    }
    else if(slot1 == false){ //however, if a ball is not found in slot 1,
      if(slot2 == false){ //but slot 2 is still empty,
        Index.spin(forward,600,rpm); //the ball in slot 3 will move up to slot 2.
      }
      else{
      Index.stop();
      }
    }
  }
  if(slot == 3){ //if the input is 3,
    if(slot3 == false){ //and there is no ball found in slot 3,
      Index.spin(forward,-600,rpm); //then the ball in the robot will come down from either slot 1 or 2 and into slot 3.
    }
    else{
      Index.stop();
    }
  }

}












/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();

  // All activities that occur before the competition starts
  // Example: clearing encoders, setting servo positions, ...
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              Autonomous Task                              */
/*                                                                           */
/*  This task is used to control your robot during the autonomous phase of   */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/

void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
}

/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/



void usercontrol(void) {
  // User control code here, inside the loop
  while (1) {
    //sets booleans (slot1,slot2,slot2space,slot3) equal to their respective line trackers, also sets what dictates true and false
  if(topBall.value(percent) <= 70){
    slot1 = true;
  }
  else{
    slot1 = false;
  }

  if(middleBall.value(percent) <= 70){
    slot2 = true;
  }
  else{
    slot2 = false;
  }

  if(middleBall2.value(percent) <= 70){
    slot2space = true;
  }
  else{
    slot2space = false;
  }

  if(intakeBall.value(percent) <= 70){
    slot3 = true;
  }
  else{
    slot3 = false;
  }


 
if(Controller1.ButtonL2.pressing()){//this says that when L2 is pressed,
  slotMove(1); //the robot will first make sure that a ball is in the slot position 1.
  shootSpeed = Index.position(degrees) + 600; //then it'll spin the Indexer for 600 degrees,
  Index.spinToPosition(shootSpeed,degrees,600,rpm); //which should be enough to shoot just 1 ball.
  slotMove(1); //then the robot will make sure that if there is any other ball being held, it will move up to slot position 1.
  wait(200,msec); //this should provide some makeup time so that the balls shoot periodically
}
else if(Controller1.ButtonL1.pressing()){ //this says that when L1 is pressed,
  Intake1.spin(forward,200,rpm); //it runs the intakes inwards
  Intake2.spin(forward,200,rpm); //to intake balls (from the goals)
  slotMove(1); //and the rest of this
  shootSpeed = Index.position(degrees) + 600; //is just the same
  Index.spinToPosition(shootSpeed,degrees,600,rpm); //as what happens
  slotMove(1); //when you press
  wait(200,msec); //Button L2
}
  else if(slot2space == true && slot1 == false){ //this bit of code is saying that if a ball is found in the gap between slot 2 and 3
    Index.spin(forward,600,rpm); //and there isn't a ball in slot 1, then the indexer should push the ball up to slot 2
  }
  else if(slot3 == true && slot2 == false && slot1 == false){ //this part of the if loop says that if a ball is in slot 3 but no ball is in slot 2,
    slotMove(2); //then move the ball up from slot 3 to slot 2
  }
  else if(slot3 == true && slot2 == true && slot1 == false){ //this part of the loop says if a ball is in slots 2 and 3,
    slotMove(1); //then move them up to slots 1 and 2
  }
  else if(slot1 == true && slot2 == false){ //this part says that if a ball is in slot 1 but there isn't a ball in slot 2,
    slotMove(2); //then move the ball down from slot 1 to slot 2
  }
//run intakes forward on button R2
else if(Controller1.ButtonR2.pressing()){
     Intake1.setVelocity(200,rpm);
     Intake2.setVelocity(200,rpm);
     Intake1.spin(forward);
     Intake2.spin(forward);
}
//run intakes backward on button R1
else if(Controller1.ButtonR1.pressing()){
     Intake1.setVelocity(-200,rpm);
     Intake2.setVelocity(-200,rpm);
     Intake1.spin(forward);
     Intake2.spin(forward);
}
//use the up button to stop the top wheel
else if(Controller1.ButtonUp.pressing()){
  Serial.stop();
}
//use the right button to spin the top wheel backwards
else if(Controller1.ButtonRight.pressing()){
  Serial.spin(forward,-200,rpm);
}
//use the left button to spin the indexer backwards
else if(Controller1.ButtonLeft.pressing()){
    Index.spin(forward,-600,rpm);
}
//if nothing is being run, then stop the intakes and Indexer. However, keep the top wheel running at 185 rpm
else{
     Intake1.stop();     
     Intake2.stop();
     Index.stop();
     Serial.spin(forward,185,rpm);
}

    //this little bit of code just says that when the down button is pressed, our motor temperature values will be displayed on the joystick
   if(Controller1.ButtonDown.pressing()){
     float backLefttemp = Ldrive1.temperature(fahrenheit);
     float frontLefttemp = Ldrive2.temperature(fahrenheit);
     float backRighttemp = Rdrive1.temperature(fahrenheit);
     float frontRighttemp = Rdrive2.temperature(fahrenheit);
     Controller1.Screen.clearScreen();
     Controller1.Screen.setCursor(2,1);
     Controller1.Screen.print(backLefttemp);
     Controller1.Screen.setCursor(1,1);
     Controller1.Screen.print( frontLefttemp);
     Controller1.Screen.setCursor(1,5);
     Controller1.Screen.print(frontRighttemp);
     Controller1.Screen.setCursor(2,5);
     Controller1.Screen.print(backRighttemp);

     wait(20,msec);
   }





    //this code is basically the same as tank drive would be on a regular drive train, but strafing is added on one of the axes.
   int Lback2 = -Controller1.Axis2.position()  + Controller1.Axis4.position();
   Ldrive2.spin(forward, Lback2, percent);

   int Rfront = -Controller1.Axis3.position() + Controller1.Axis4.position();
   Rdrive1.spin(forward, Rfront, percent);

   int Lfront = -Controller1.Axis4.position() - Controller1.Axis2.position();
   Ldrive1.spin(forward, Lfront, percent);

   int Rback = -Controller1.Axis3.position() - Controller1.Axis4.position();
   Rdrive2.spin(forward, Rback, percent);


       wait(20, msec); // Sleep the task for a short amount of time to
                    // prevent wasted resources.
  }

}

//
// Main will set up the competition functions and callbacks.
//
int main() {
  // Set up callbacks for autonomous and driver control periods.
  Competition.autonomous(autonomous);
  Competition.drivercontrol(usercontrol);

  // Run the pre-autonomous function.
  pre_auton();

  // Prevent main from exiting with an infinite loop.
  while (true) {
    wait(100, msec);
  }
}
