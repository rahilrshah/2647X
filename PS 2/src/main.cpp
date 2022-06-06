/*----------------------------------------------------------------------------*/
/*                                                                            */
/*    Module:       main.cpp                                                  */
/*    Author:       VEX                                                       */
/*    Created:      Thu Sep 26 2019                                           */
/*    Description:  Competition Template                                      */
/*                                                                            */
/*----------------------------------------------------------------------------*/

// ---- START VEXCODE CONFIGURED DEVICES ----
// ---- END VEXCODE CONFIGURED DEVICES ----

#include "vex.h"

using namespace vex;

// A global instance of competition
competition Competition;

// define your global instances of motors and other devices here

/*---------------------------------------------------------------------------*/
/*                          Pre-Autonomous Functions                         */
/*                                                                           */
/*  You may want to perform some actions before the competition starts.      */
/*  Do them in the following function.  You must return from this function   */
/*  or the autonomous and usercontrol tasks will not be started.  This       */
/*  function is only called once after the V5 has been powered on and        */
/*  not every time that the robot is disabled.                               */
/*---------------------------------------------------------------------------*/
bool inertCalibrated=false;


void calibrateInert() {
  wait(1000,msec);
  if(inertCalibrated==false) {
    inert.calibrate();
    while(inert.isCalibrating()==true) {
      wait(10,msec);
    }
    inertCalibrated=true;
    wait(500,msec);


    printf("INERT CALIBRATED1\n");
    Controller1.rumble("..");
  } 
}

void pre_auton(void) {
  // Initializing Robot Configuration. DO NOT REMOVE!
  vexcodeInit();
  claw.open();
  calibrateInert();

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


/*---------------------------------------------------------------------------*/
/*                                                                           */
/*                              User Control Task                            */
/*                                                                           */
/*  This task is used to control your robot during the user control phase of */
/*  a VEX Competition.                                                       */
/*                                                                           */
/*  You must modify the code to add your own robot specific commands here.   */
/*---------------------------------------------------------------------------*/


//robotcontrol----------------------------------------------------------------------------------------------------------------------------------------------------------
int robotControl::shiftMode=0;
bool robotControl::volt_enabled=true;
  //holds the position of the two shift gears

bool robotControl::driveEngaged = true;
bool robotControl::armEngaged = false;

  //is true when the robot is in an active shift
bool robotControl::shifting = false;
bool robotControl::driveBoost = true;
double robotControl::shiftTimer = 0;
  //holds the final desired shifting state (prevents sus stuff)
int robotControl::shiftTarget = 1;
double robotControl::armTorque = 100;

  //represents the intention of the shifting system, what it wants the final state to be once it is done, not the actual current state
  //0-engage drive
  //1 engage driveTrain
  //2 engage both(sus)
int robotControl::transmissionState = 0;
int robotControl::transmissionState_last = 0;

//0peg intaking
//1arm down peg down
//2neutral peg up free arm movement
//3score in front free arm movement
//4score in tall preset arm movement
double robotControl::flippyPid_speed=100;
double robotControl::armPid_speed=100;

int robotControl::mechTGT=2;
bool robotControl::driveHold=false;
int robotControl::mechMode=1;
bool robotControl::driverControl = true;
double robotControl::leftDrive = 0;
double robotControl::rightDrive = 0;
double robotControl::armSpeed = 0;
double robotControl::lastArmSpeed = 0;
double robotControl::armHoldPosR = 0;
double robotControl::armHoldPosL = 0;
//val of 200 to coast;
bool robotControl::armPIDenabled=true;
bool robotControl::flippyPIDenabled=false;
double robotControl::armTGT = 0;
double robotControl::flippyTGT = 0;
bool robotControl::rezero_flippy=false;

double robotControl::manualPegSpeed=0;
double robotControl::manualArmSpeed=0;
// ODOM --------------------------------------------------------------------------------------------------------------------------------------------------------------------
int odom::prevLocation=0;
int odom::rotations=0;
double odom::pitch_z=0;
double odom::roll_z=0;
double odom::odomTimer=0;
double odom::odomAngle=0;
double odom::odomAngle_last=0;


double odom::encoder_left_last=0;
double odom::encoder_right_last=0;
double odom::encoder_back_last=0;

double odom::delta_x=0;
double odom::delta_y=0;
double odom::x_position=0;
double odom::y_position=0;

//bez hell-----------------------------------------------------------------------------------------------------------------------------------------------------------------------
double bez::angTol=20;

double bez::purePursuitTarget_x = 0;
double bez::purePursuitTarget_y = 0;

int bez::purePursuit_point = 1;
double bez::purePursuit_pathDist = 0;
double bez::point_Dist = 0;
bool bez::purePursuit_final = false;
double bez::purePursuit_error_last= 0;
bool bez::purePursuit_initialTurn = true;

double bez::PIDstartTime=0;
double bez::turnIntegral=0;
double bez::Perror=0;



bool lastB = false;
bool lastX = false;
bool lastY = false;
bool lastA = false;
bool lastR1 = false;
bool lastR2 = false;
bool lastL1 = false;
bool lastL2 = false;

bool lastUp = false;
bool lastDown = false;
bool lastRight = false;
bool lastLeft = false;

void lastButtons() {
  lastB = Controller1.ButtonB.pressing();
  lastX = Controller1.ButtonX.pressing();
  lastY = Controller1.ButtonY.pressing();
  lastA = Controller1.ButtonA.pressing();
  lastR1 = Controller1.ButtonR1.pressing();
  lastR2 = Controller1.ButtonR2.pressing();
  lastL1 = Controller1.ButtonL1.pressing();
  lastL2 = Controller1.ButtonL2.pressing();
  lastUp = Controller1.ButtonUp.pressing();
  lastDown = Controller1.ButtonDown.pressing();
  lastLeft = Controller1.ButtonLeft.pressing();
  lastRight = Controller1.ButtonRight.pressing();
}

double conveyorSpd = 0;




bool clawState =false;
bool rearMogoState = false;
bool pegState = false;



bool initialized = false;
int background() {
  double stalled = -300;
  while(true) {
    initialized=true;
    robotControl::controlShift();
    if (rearMogoState==true) {
      rearMogo.open();
    } else {
      rearMogo.close();
    }
    if(clawState==true) {
      claw.open();
    } else {
      claw.close();
    }
    wait(20,msec);
  }
  return 0;

}

int flippyRunner() {
  robotControl::flippyPID();
  return 0;
}

int armRunner() {
  robotControl::armPID();
  return 0;
}

int mechRunner() {
  robotControl::mechControl();
  return 0;
}

int odomRunner() {
  while(true) {
    odom::updateOdom();
    wait(20,msec);
  }
  return 0;
}

int conveyorRunner() {
  while(true) {
    if(conveyor.torque()>0.2&&abs(conveyor.velocity(rpm))<100){
      conveyor.spin(forward, -300, rpm);
      wait(200,msec);
    } else {
      conveyor.spin(forward,conveyorSpd,rpm);
    }
    wait(20,msec);
  }
}

double lastSlew = 0;

double slew(double rate, double input, double lastInput) {
  double step = rate*(0.02);

  if (fabs(input-lastInput)<step) {
    return input;
  } else {
    if (input>lastInput) {
      return lastInput+step;
    } else {
      return lastInput-step;
    }
  }
}


int rightTriggerMode = 0;

int rightTrigger() {
  double startWait=0;
  bool lR1 = false;
  bool lR2 = false;
  while (true) {  
    if(rightTriggerMode==3&&(Controller1.ButtonR1.pressing()==true||Controller1.ButtonR2.pressing()==true)){
      //nothing
    } else if((Controller1.ButtonR1.pressing()!=lR1)||(Controller1.ButtonR2.pressing()!=lR2)) {
      startWait=Brain.Timer.time(msec);
      while((Brain.Timer.time(msec)-startWait)<80) {
        wait(20,msec);
        if((Controller1.ButtonR1.pressing()==Controller1.ButtonR2.pressing())) {
          break;
        }
      }

      if(Controller1.ButtonR1.pressing()==false&&Controller1.ButtonR2.pressing()==false) {
        rightTriggerMode=0;
      } else if(Controller1.ButtonR1.pressing()==true&&Controller1.ButtonR2.pressing()==false) {
        rightTriggerMode=1;
      } else if(Controller1.ButtonR1.pressing()==false&&Controller1.ButtonR2.pressing()==true) {
        rightTriggerMode=2;
      } else if(Controller1.ButtonR1.pressing()==true&&Controller1.ButtonR2.pressing()==true) {
        rightTriggerMode=3;
      }
    }
    lR1 = Controller1.ButtonR1.pressing();
    lR2 = Controller1.ButtonR2.pressing();
    wait(20,msec);
  }
  return 0; 
}
double visionTGT = 0;
bool bigSplit = false;
bool rearVis=true;
//0 = blue
//1 = red]
//2 = yellow(neutral)
//if reverse color is false, the robot starts on the side with the red platoform
bool reverseColor=true;
int colorMode = 0;
int visionRunner() {
  double ang = 0;
  int width = 0;
  int height = 0;

  while(true) {
    if(rearVis==true) {
      //use the rear vision sensor
    if(colorMode==0) {
      if(reverseColor==false) {
        VIS_REAR.takeSnapshot(VIS_REAR__BLU,2);
      } else {
        VIS_REAR.takeSnapshot(VIS_REAR__RD,2);
      }
    } else if(colorMode==1) {
      if(reverseColor==false) {
        VIS_REAR.takeSnapshot(VIS_REAR__RD,2);
      } else {
        VIS_REAR.takeSnapshot(VIS_REAR__BLU,2);
      }
    } else if(colorMode==2) {

      VIS_REAR.takeSnapshot(VIS_REAR__YELLW,2);
    }

    if(VIS_REAR.objectCount==1) {
      ang = VIS_REAR.largestObject.centerX;
      height = VIS_REAR.largestObject.height;
      width = VIS_REAR.largestObject.width;
      bigSplit=false;

      if(height*width>400) {
        visionTGT = ang-160;
      } else {
        visionTGT=0;
      }
    } else if(VIS_REAR.objectCount>1) {
      if(((VIS_REAR.objects[0].width*VIS_REAR.objects[0].height)>400)&&((VIS_REAR.objects[1].width*VIS_REAR.objects[1].height)>400)) {
        ang = (VIS_REAR.objects[0].centerX+VIS_REAR.objects[1].centerX)/2;
        visionTGT = ang-160;
        bigSplit=true;
      } else {
        bigSplit=false;
        ang = VIS_REAR.largestObject.centerX;
        height = VIS_REAR.largestObject.height;
        width = VIS_REAR.largestObject.width;
        if(height*width>400) {
          visionTGT = ang-160;
        } else {
          visionTGT=0;
        }
      }
    } else {
      bigSplit=false;
      ang=0;
      height = 0;
      width=0;
      visionTGT=0;
    }
    } else {
           //use the front vision sensor
    if(colorMode==0) {
      if(reverseColor==false) {
        VIS_F.takeSnapshot(VIS_F__BLU,2);
      } else {
        VIS_F.takeSnapshot(VIS_F__RD,2);
      }
    } else if(colorMode==1) {
      if(reverseColor==false) {
        VIS_F.takeSnapshot(VIS_F__RD,2);
      } else {
        VIS_F.takeSnapshot(VIS_F__BLU,2);
      }
    } else if(colorMode==2) {
      VIS_F.takeSnapshot(VIS_F__YELLW,2);
    }

    if(VIS_F.objectCount==1) {
      ang = VIS_F.largestObject.centerX;
      height = VIS_F.largestObject.height;
      width = VIS_F.largestObject.width;
      bigSplit=false;

      if(height*width>400) {
        visionTGT = ang-160;
      } else {
        visionTGT=0;
      }
    } else if(VIS_F.objectCount>1) {
      if(((VIS_F.objects[0].width*VIS_F.objects[0].height)>400)&&((VIS_F.objects[1].width*VIS_F.objects[1].height)>400)) {
        ang = (VIS_F.objects[0].centerX+VIS_F.objects[1].centerX)/2;
        visionTGT = ang-160;
        bigSplit=true;
      } else {
        bigSplit=false;
        ang = VIS_F.largestObject.centerX;
        height = VIS_F.largestObject.height;
        width = VIS_F.largestObject.width;
        if(height*width>400) {
          visionTGT = ang-160;
        } else {
          visionTGT=0;
        }
      }
    } else {
      bigSplit=false;
      ang=0;
      height = 0;
      width=0;
      visionTGT=0;
    }
 
    }




    wait(20,msec);
  }
  return 0;
}


int diagnostic() {


  while(true) {
    // leftEncoder = (0.70588235)*((Ldrive1.position(degrees)+Ldrive2.position(degrees))/2);
    // rightEncoder = (0.70588235)*((Rdrive1.position(degrees)+Rdrive2.position(degrees))/2);
    // distanceDriven = ((leftEncoder+rightEncoder)/2)*0.0242261104;

    // printf("armSpeed: %lf\n", robotControl::armSpeed);
    // printf("distance: %lf\n", distanceDriven);
    // printf("flippyTGT: %lf\n", robotControl::flippyTGT);

    printf("flippypos: %lf\n", robotControl::getFlippyPos());
    // printf("flippy torque: %lf \n", flippy.torque());
    printf("arm: %lf\n", robotControl::getArmPos());
    // printf("armTorR: %lf\n", Rdrive3_arm.torque());
    // printf("armTorl: %lf\n", Ldrive3_arm.torque());
    // printf("armTGT: %lf\n", robotControl::armTGT);
    // printf("mechMode: %d\n", robotControl::mechMode);
    // printf("mechTGT: %d\n", robotControl::mechTGT);
    // printf("flippyTGT: %lf\n", robotControl::flippyTGT);
  //   printf("LEFT: %lf\n", robotControl::leftDrive);
  //   printf("RIGHT: %lf\n", robotControl::rightDrive);
  //   printf("convSpeed: %lf\n", conveyorSpd);
    printf("trueHeading: %lf\n", odom::trueHeading());
    printf("x_position: %lf\n", odom::x_position);
  printf("y_position: %lf\n", odom::y_position);
  printf("VISION TGT: %lf\n", visionTGT);
  printf("line1: %ld\n", line1.value(percent));
  printf("inert.pitch: %lf \n", inert.pitch());
    printf("---------------------------------------\n");
    // // printf("flippyPIDenabled: ");
    // if(robotControl::flippyPIDenabled==true) {
    //   printf("true\n");
    // } else {
    //   printf("false\n");
    // }

    wait(1000,msec);
  }
  return 0;
}

void motion1() {
  robotControl::flippyPIDenabled=true;
  robotControl::flippyTGT=45;
  robotControl::armPIDenabled=false;
  robotControl::armSpeed=0;
  rearMogoState=true;
  wait(300,msec);

  while(true) {
    bez::setDrive_tank_abs(-35, 270);
    wait(20,msec);
    if(odom::x_position>9) {
      break;
    }
  }
  rearMogoState=false;
  wait(100,msec);
  bez::setDrive_Tank_raw(0, 0);
  double timer2  =Brain.Timer.time(msec);

  while(Brain.Timer.time(msec)-timer2<450) {
    bez::setDrive_tank_abs(35, -45);
    wait(20,msec);
  }
  //clear wall
  //begin path


  double point3[2] = {12, 48};
  double point4[2] = {12, 72};
  double point5[2] = {32.5, 106};
  bez::scuffedPID(point3, 400, 1, 0, 0, 0, 0, false);


  double **path =bez::addArrays(bez::generateStraightPath(point3, 50), bez::BezierArray(point3, point4, point5,50));
  //initial settings

  double pow=0;
  double lastPow = 00;
  double spd = 0;
  double followDist = 0;
  bez::resetPurePursuit();
  while(true) {
    if(odom::y_position<35) {
      spd=80;
      followDist=14;
    } else if(odom::y_position<53) {
      spd=50;
      followDist=14;
    } else if(odom::y_position<102) {
      followDist=14;
      spd=45;
      clawState=true; 
      robotControl::armPIDenabled=true;
      robotControl::armTGT = 115;
    } else {
      followDist=14;
      spd=50;
      clawState=true; 
      robotControl::armPIDenabled=true;
      robotControl::armSpeed=70;
      robotControl::armTGT = 65;
    }
    pow = slew(200, spd, lastPow);
    if((bez::purePursuit(path, followDist, pow, 10, true, false)==true)) {
      break; 
    }
    lastPow = pow;
    wait(20,msec);
  }
  robotControl::armPid_speed=70;
  robotControl::armTGT=65;
  bez::setDrive_Tank_raw(0,0);
  wait(400,msec);
  clawState=false;
  wait(0,msec);

  double y1 = 0;
  robotControl::armPid_speed=100;

  while(true) {
    if(y1==0&&(line1.value(percent)<15)) {
      y1=odom::y_position;
    }

    if(odom::y_position>(y1)) {
      spd=-60;
    } else {
      spd=0;
      if(pow==0) {
        break;
      }
    }
    pow = slew(200, spd, lastPow);
    bez::setDrive_tank_abs(pow,30);
    lastPow = pow;
    wait(20,msec);
  }
  robotControl::flippyTGT=20;
  robotControl::armTGT=0;
  bez::setDrive_Tank_raw(0, 0);
  wait(100,msec);
  bez::scuffedPID(90, 700, 1, 0, 0, 0, 0);
}


void motion2() {
  robotControl::driveBoost=false;

  double pow=0;
  double lastPow = 00;
  double spd = 15;

  bez::resetPurePursuit();
  rearMogoState=true;
  robotControl::mechTGT=0;
  robotControl::driveBoost=false;
  while(true) {

    if(odom::x_position<62) {
      if(robotControl::mechMode==0) {
        conveyorSpd=600;
        spd=30;
      }
    } else {
      spd=0;
      if(pow==0) {
        break;
      }
    }
    pow = slew(150, spd, lastPow);
    bez::setDrive_tank_abs(pow,90);
    lastPow = pow;
    wait(20,msec);
  }

  // while(true) {
  //   if(odom::x_position>60) {
  //     conveyorSpd=600;
  //   } else {
  //     conveyorSpd=0;
  //     robotControl::mechTGT=5;
  //     robotControl::armPid_speed=100;
  //     robotControl::flippyPid_speed=100;
  //     robotControl::armTGT=0;
  //     robotControl::armPIDenabled=true;
  //     //prep to score high--------------------------------------------------------------
  //     robotControl::flippyPIDenabled=true;
  //     robotControl::flippyTGT=60;
  //     armAct.open();
  //   }
  //   if(odom::x_position>50) {
  //     spd=-65;
  //   } else {
  //     spd=0;
  //     if(pow==0) {
  //       break;
  //     }
  //   }
  //   pow = slew(200, spd, lastPow);
  //   bez::setDrive_tank_abs(pow,90);
  //   lastPow = pow;
  //   wait(20,msec);
  // }
  double tgt = 46;
  double timer1 = 999999;
  while(true) {
    if(odom::x_position>57) {
      conveyorSpd=600;
    } else {
      conveyorSpd=0;
      robotControl::mechTGT=5;
      robotControl::armPid_speed=100;
      robotControl::flippyPid_speed=100;
      robotControl::armTGT=0;
      robotControl::armPIDenabled=true;
      //prep to score high--------------------------------------------------------------
      robotControl::flippyPIDenabled=true;
      robotControl::flippyTGT=60;
      armAct.open();
    }
    if((abs(tgt-odom::x_position)<1.)&&timer1==999999) {
      timer1= Brain.Timer.time(msec);
    }
    if(((Brain.Timer.time(msec)-timer1))>200) {
      break;
    }
    spd = (tgt-odom::x_position)*5;
    if(spd<-50) {
      spd=-50;
    }
    pow = slew(200, spd, lastPow);
    bez::setDrive_tank_abs(pow,90);
    lastPow = pow;
    wait(20,msec);
  }
  armAct.open();
  bez::setDrive_Tank_raw(0,0);
  wait(200,msec);
  printf("696969:%lf\n", odom::x_position);
  bez::scuffedPID(0, 700, 0.7, 0, 0, 0, 0);
  
  printf("4324:%lf\n", odom::x_position);
  armAct.open();
  robotControl::armPIDenabled=true;
  robotControl::armTGT=115;
}

void motion3() {
  //drive backwards to inake high tower
  printf("333323124:%lf\n",odom::x_position);

  double point1[2] = {odom::x_position,odom::y_position};
  double point2[2] = {51, 65};
  double point3[2] = {51, 65};

  double **path =bez::BezierArray(point1, point2, point3, 50);
  //initial settings

  double pow=0;
  double lastPow = 00;
  double spd = 0;
  // double followDist = 0;
  // bez::resetPurePursuit();
  // while(true) {

  //   spd=30;
  //   followDist=12;
  //   if(odom::y_position<70) {
  //     rearMogoState=false;
  //   }
  //   pow = slew(100, spd, lastPow);
  //   if((bez::purePursuit(path, followDist, pow, 0, true, true)==true)) {
  //     break; 
  //   }
  //   lastPow = pow;
  //   wait(20,msec);
  // }
  colorMode=2;
  rearVis=true;
    while(true) {

    if(odom::y_position<69) {
      rearMogoState=false;
    }
    wait(20,msec);
    if(odom::y_position>73) {
      bez::setDrive_tank(-20, visionTGT*0.13);
    } else if(odom::y_position>66) {
      bez::setDrive_Tank_raw(-25, -25);
    } else {
      break;
    }
  }

  double y2 = -999;
  //drive backwards while loading high tower
  lastPow=-25;
  while(true) {
    if(odom::y_position<56) {
      robotControl::mechTGT=4;
    }
    if((line1.value(percent)<15&&y2==-999)&&odom::y_position<50) {
      y2=odom::y_position;
    }
    if(odom::y_position>35) {
      pow = slew(50, spd, lastPow);
    } else {
      pow = slew(100, spd, lastPow);

    }
    lastPow = pow;
    if(odom::y_position>y2-31) {
      spd=-57;
    } else {
      spd=0;
      if(pow==0) {
        break;
      }
    }
    bez::setDrive_tank_abs(pow,-45);
    wait(20,msec);
  }
  //drop off high tower
  double monke = Brain.Timer.time(msec);
    while(true) {
    if(odom::y_position<(y2-30)) {
      pow = slew(350, spd, lastPow);
    } else {
      pow = slew(150, spd, lastPow);
    }
    lastPow = pow;
    if((Brain.Timer.time(msec)-monke)>250) {
      rearMogoState=true;
    } 
    if(odom::y_position<(y2-27)) {
      spd=50;
    } else {
      robotControl::mechTGT=5;
      robotControl::armPIDenabled=true;
      robotControl::armTGT=10;
      robotControl::flippyPIDenabled=true;
      robotControl::flippyTGT=50;
      spd=0;
      if(pow==0) {
        break;
      }
    }
    bez::setDrive_tank_abs(pow,-45);
    wait(20,msec);
  }
  robotControl::mechTGT=5;
  bez:bez::setDrive_Tank_raw(0,0);
  wait(100,msec);
  robotControl::armPIDenabled=true;
  robotControl::armTGT=10;
  rearVis=true;
  colorMode=0;
  bez::scuffedPID(-90, 600, 0.8, 0, 0, 0, 0);

}   

void motion4() {
  //reverse and grab 2i
  robotControl::driveBoost=true;
  robotControl::armPIDenabled=false;
  robotControl::armSpeed=0;
  robotControl::flippyPIDenabled=true;
  robotControl::flippyTGT=30;

    double pow=0;
    double lastPow = 00;
    double spd = 0;
    while(true) {
    pow = slew(200, spd, lastPow);
    lastPow = pow;

    if(odom::x_position<100.5) {
      spd=-50;
    } else {
      spd=0;
      if(pow==0) {
        break;
      }
    }
    bez::setDrive_tank(pow, visionTGT*0.2);
    wait(20,msec);
  }
  bez::setDrive_Tank_raw(-30, -30);
  wait(150,msec);
  bez::setDrive_Tank_raw(0,0);
  rearMogoState=false;
  bez::setDrive_Tank_raw(50, 50);
  wait(180,msec);
  double tgt = 67;
  double timer1 = 1234567;
  // while(true) {
  //   if((abs(tgt-odom::x_position)<1.5)&&timer1==999999) {
  //     timer1= Brain.Timer.time(msec);
  //   }
  //   if(((Brain.Timer.time(msec)-timer1))>50) {
  //     break;
  //   }
  //   spd = (odom::x_position-tgt)*5;

  //   pow = slew(200, spd, lastPow);
  //   bez::setDrive_tank_abs(pow,-90);
  //   lastPow = pow;
  //   wait(20,msec);
  // }
  bez::scuffedPID(-5, 600, 0.9, 0, 0, 0, 0);
    while(true) {
    conveyorSpd=600;
    if((abs(tgt-odom::y_position)<2)&&timer1==1234567) {
      timer1= Brain.Timer.time(msec);
    }
    if(((Brain.Timer.time(msec)-timer1))>50) {
      break;
    }
    spd = (tgt-odom::y_position)*6;
    if(spd>60) {
      spd=60;
    } 
    pow = slew(300, spd, lastPow);
    bez::setDrive_tank_abs(pow, 0);
    lastPow = pow;
    wait(20,msec);
    colorMode=2;
    rearVis=false;
  }



  double timer2 = Brain.Timer.time(msec);
  armAct.close();
  tgt=55;
  timer1 = 999999;
  while(true) {
    conveyorSpd=600;
    if((abs(tgt-odom::y_position)<1)&&timer1==999999) {
      timer1= Brain.Timer.time(msec);
    }
    if(((Brain.Timer.time(msec)-timer1))>100) {
      break;
    }
    spd = (tgt-odom::y_position)*8;
    if(spd<-70) {
      spd=-70;
    } 
    pow = slew(300, spd, lastPow);
    bez::setDrive_tank_abs(pow, -40);
    lastPow = pow;
    wait(20,msec);
    colorMode=2;
    rearVis=false;
  }
  robotControl::armPIDenabled=true;
  robotControl::armTGT=0;

  while(true) {
    if(visionTGT==0) {
      bez::setDrive_Tank_raw(-40,40);
    } else {
      bez::setDrive_tank(0, visionTGT*0.2);
      if(abs(visionTGT)<30) {
        break;
      }
    }
    wait(20,msec);
  }
  bool flag=false;
  while(true) {
    if(claw_range.objectDistance(mm)<40&&claw_range.objectDistance(mm)!=0){
      break;
    }
    if(robotControl::getArmPos()<5) {
      flag=true;
    }

    if(flag==false) {
      robotControl::armPIDenabled=true;
      robotControl::armTGT=0;
    } else {
      robotControl::armPIDenabled=false;
      robotControl::armSpeed=10;
    }
    bez::setDrive_tank(30, visionTGT*0.2);
    wait(20,msec);
    robotControl::armPIDenabled=true;
    robotControl::armTGT=0;
  }
  bez::setDrive_Tank_raw(30, 30);
  clawState=true;

}

void motion5() {
  //drop off second neutral
  double point1[2] = {odom::x_position,odom::y_position};
  double goalTGT[2] = {56, 117};
  robotControl::armPIDenabled=true;
  robotControl::armTGT=74;
  bez::scuffedPID(goalTGT, 700, 1, 0, 0, 0, 0, false);

  double **path =bez::generateStraightPath(bez::goalAdj(goalTGT, 15), 50);
  //initial settings

  double pow=0;
  double lastPow = 00;
  double spd = 0;
  double followDist = 0;
  bez::resetPurePursuit();
  while(true) {
    followDist=14;
    spd=55;
    conveyorSpd=600;

    pow = slew(250, spd, lastPow);
    if((bez::purePursuit(path, followDist, pow, 10, true, false)==true)) {
      break; 
    }

    if(bez::distanceTo(goalTGT[0], goalTGT[1])<27) {
      robotControl::armTGT=65;
    }
    if(bez::distanceTo(goalTGT[0], goalTGT[1])<16) {
      clawState=false;
    }
    lastPow = pow;
    wait(20,msec);
  }
  clawState=false;
  bez::setDrive_Tank_raw(0, 0);
  wait(100,msec);
}

void motion6() {
  double pow=0;
  double spd=0;
  double lastPow=0;
  double inity= odom::y_position;
  //drive backwards
  while(true) {

    if(odom::y_position>inity-7) {
      spd=-50;
    } else {
      spd=0;
      if(pow==0) {
        break;
      }
    }
    bez::setDrive_tank_abs(pow, -25);
    pow = slew(150, spd, lastPow);
    lastPow = pow;
    wait(20,msec);
  }
  bez::setDrive_Tank_raw(0, 0);
  rearMogoState=true;
  //drive forwards clear of the mogo

  double y_tgt=odom::y_position+4;
  double timer1 = 999999;
  while(true) {
    conveyorSpd=00;
    if((abs(y_tgt-odom::y_position)<1)&&timer1==999999) {
      timer1= Brain.Timer.time(msec);
    }
    if(((Brain.Timer.time(msec)-timer1))>100) {
      break;
    }
    spd = (y_tgt-odom::y_position)*7;
    if(spd>40) {
      spd=40;
    } 
    pow = slew(100, spd, lastPow);
    bez::setDrive_Tank_raw(pow, pow);
    lastPow = pow;
    wait(20,msec);
  }
  rearVis=false;
  colorMode= 0;
  double angle1 = odom::trueHeading();
  robotControl::armTGT=0;
  bez::setDrive_Tank_raw(-80, 80);
  wait(200,msec);
  bez::scuffedPID(angle1+180, 700, 1, 0, 0, 0, 0);
  //turn towards mogo using heading
  // while(true) {
  //   robotControl::armPIDenabled=true;
  //   robotControl::armTGT=0;
  //   if(odom::trueHeading()>270) {
  //     bez::setDrive_Tank_raw(-50, 50);

  //   }else if(odom::trueHeading()>210) {
  //     bez::setDrive_Tank_raw(-40, 40);
  //   } else {
  //     break;
  //   }
  //   wait(20,msec);
  // }
  //use vision sensor to grab
  bool flag = false;
  while(true) {
    if(robotControl::getArmPos()<5) {
      flag=true;
    }

    if(flag==false) {
      robotControl::armPIDenabled=true;
      robotControl::armTGT=0;
    } else {
      robotControl::armPIDenabled=false;
      robotControl::armSpeed=10;
    }
    if(visionTGT==0) {
      bez::setDrive_Tank_raw(-40, 40);
    } else {
      bez::setDrive_tank(0, visionTGT*0.2);
      if(abs(visionTGT)<40) {
        break;
      }
    }
    wait(20,msec);
  }
  while(true) {

    if(claw_range.objectDistance(mm)<60&&claw_range.objectDistance(mm)!=0) {
      break;
    }
    bez::setDrive_tank(40, visionTGT*0.20);
    wait(20,msec);
  }
  bez::setDrive_Tank_raw(40, 40);
  clawState=true;
  wait(100,msec);
}

void motion7() {
  //drop off first blue
  double point1[2] = {odom::x_position,odom::y_position};
  double goalTGT[2] = {66, 116};
  robotControl::armPIDenabled=true;
  robotControl::armTGT=77;
  bez::scuffedPID(goalTGT, 1000, 1, 0, 0, 0, 0, false);

  //initial settings
  double **path =bez::generateStraightPath(bez::goalAdj(goalTGT, 15), 50);
  double pow=0;
  double lastPow = 00;
  double spd = 0;
  double followDist = 15;
  bez::resetPurePursuit();
  while(true) {
    spd=75;
    followDist = 15;
    pow = slew(250, spd, lastPow);
    if((bez::purePursuit(path, followDist, pow, 17, true, false)==true)) {
      break; 
    }
    if(bez::distanceTo(goalTGT[0], goalTGT[1])<30) {
      robotControl::armTGT=55;
    }
    if(bez::distanceTo(goalTGT[0], goalTGT[1])<15) {
      clawState=false;
    }
    lastPow = pow;
    wait(20,msec);
  }
  bez::setDrive_Tank_raw(0, 0);
  clawState=false;
  wait(200,msec);
  lastPow=0;
  pow=0;
  //drive back
  while(true) {

    if(odom::y_position>97) {
      spd=-60;
    } else {
      spd=0;
      if(pow==0) {
        break;
      }
    }
    bez::setDrive_Tank_raw(pow,pow);    
    pow = slew(250, spd, lastPow);
    lastPow = pow;
    wait(20,msec);
  }
  bez::setDrive_Tank_raw(0, 0);
}

void motion8() {
  robotControl::armPIDenabled=true;
  robotControl::armTGT=10;
  //turn to grab second blue
  bez::scuffedPID(-90, 700, 1, 0, 0, 0, 0);
  double spd=80;
  double lastPow=0;
  double pow=0;
  while(true) {
    conveyorSpd=600;
    pow = slew(250, spd, lastPow);
    lastPow = pow;
    if(visionTGT==0) {
      bez::setDrive_tank_abs(pow, -90);
    } else {
      spd=60;
      bez::setDrive_tank(pow, visionTGT*0.2);
    }
    if(odom::x_position<40) {
      robotControl::armTGT=0;
    }
    if(claw_range.objectDistance(mm)<60&&claw_range.objectDistance(mm)!=0) {
      break;
    }
    wait(20,msec);
  }
  conveyorSpd=0;


  clawState=true;
  while(true) {
    conveyorSpd=0;
    spd=0; 
    pow = slew(300, spd, lastPow);
    lastPow = pow;  
    bez::setDrive_Tank_raw(pow, pow);
    wait(20,msec);
    if(pow==0) {
      break;
    }
  }
  bez::setDrive_Tank_raw(0, 0);
  wait(100,msec);
}

void motion9() {
  //drop off second blue
  double point1[2] = {odom::x_position,odom::y_position};
  double goalTGT[2] = {26, 120};
  robotControl::armPIDenabled=true;
  robotControl::armTGT=100;
  bez::scuffedPID(goalTGT, 1000, 1, 0, 0, 0, 0, false);

  //initial settings
  double **path =bez::generateStraightPath(bez::goalAdj(goalTGT, 15), 50);
  double pow=0;
  double lastPow = 00;
  double spd = 0;
  double followDist = 10;
  bez::resetPurePursuit();
  while(true) {
    conveyorSpd=0;
    spd=75;
    pow = slew(250, spd, lastPow);
    if((bez::purePursuit(path, followDist, pow, 22, true, false)==true)) {
      break; 
    }
    if(bez::distanceTo(goalTGT[0], goalTGT[1])<30) {
      robotControl::armTGT=60;
    }
    if(bez::distanceTo(goalTGT[0], goalTGT[1])<18) {
      clawState=false;
    }
    lastPow = pow;
    wait(20,msec);
  }
  clawState=false;
  double yTGT=odom::y_position;
  lastPow=0;
  pow=0;
  //drive back
  while(true) {

    if(odom::y_position>yTGT-1.5) {
      spd=-40;
    } else {
      spd=0;
      if(pow==0) {
        break;
      }
    }
    bez::setDrive_Tank_raw(pow,pow);    
    pow = slew(200, spd, lastPow);
    lastPow = pow;
    wait(20,msec);
  }
  rearVis=true;
  colorMode=1;

  bez::scuffedPID(80, 700, 1, 0, 0, 0, 0);
  while(odom::x_position>-2) {
    if(visionTGT==0) {
      bez::setDrive_tank_abs(-50, 80);
    } else {
      bez::setDrive_tank(-50, visionTGT*0.2);
    }
    wait(20,msec);
  }
  bez::setDrive_Tank_raw(-60, -60);
  wait(300,msec);
  rearMogoState=false;

}

void motion10() {
  double point2[2] = {60,84};
  double point3[2] = {85, 110};
  robotControl::armPIDenabled=true;
  robotControl::armTGT=7;
  bez::scuffedPID(point2, 300, 1, 0, 0, 0, 0, false);
  double point1[2] = {odom::x_position,odom::y_position};
  robotControl::armPIDenabled=false;
  robotControl::armSpeed=0;
  //initial settings
  double **path =bez::BezierArray(point1, point2, point3, 50);
  double pow=0;
  double lastPow = 00;
  double spd = 0;
  double followDist = 14;
  bez::resetPurePursuit();

  while(true) {
    conveyorSpd=600;
    spd=90;
    pow = slew(250, spd, lastPow);
    if((bez::purePursuit(path, followDist, pow, 8, true, false)==true)) {
      break; 
    }
    lastPow = pow;
    wait(20,msec);
    rearVis=false;
    colorMode=1;
  }
}

void motion11() {

  robotControl::armPIDenabled=true;
  robotControl::armTGT=0;
  //grab second red
  double inithead = odom::trueHeading();
  bez::scuffedPID(-60, 700, 1, 0, 0, 0, 0);
  while(true) {
    if(visionTGT==0) {
      bez::setDrive_Tank_raw(-40,40);
    } else {
      bez::setDrive_tank(0, visionTGT*0.2);
      if(abs(visionTGT)<30) {
        break;
      }
    }
    wait(20,msec);
  }

  double lastPow=0;
  double pow=0;
  double spd=0;

  while(true) {
    conveyorSpd=600;
    if(visionTGT==0) {
      spd=50;
    } else {
      break;
    }
    
    bez::setDrive_Tank_raw(pow, pow);    
    pow = slew(200, spd, lastPow);
    lastPow = pow;
    wait(20,msec);
  }

  while(true) {
    bez::setDrive_tank(30, visionTGT*0.20);
    wait(20,msec);
    if(claw_range.objectDistance(mm)<40&&claw_range.objectDistance(mm)!=0) {
      break;
    }
  }
  clawState=true;
  bez::setDrive_Tank_raw(30, 30);
  double yTGT=odom::y_position;
  double initPoint[2] = {odom::x_position, odom::y_position};

  lastPow=0;
  pow=0;
  spd=0;

  while(true) {
    if(bez::distanceTo(initPoint[0], initPoint[1])<4) {
      spd=-50;
    } else {
      robotControl::armPIDenabled=false;
      robotControl::armSpeed=0;
      spd=0;
      break;
    }
    if(bez::distanceTo(initPoint[0], initPoint[1])>2) {
      robotControl::armPIDenabled=true;
      robotControl::armTGT=15;
    }
    bez::setDrive_tank_abs(pow, -45);    
    pow = slew(300, spd, lastPow);
    lastPow = pow;
    wait(20,msec);
  }
  robotControl::armPIDenabled=true;
  robotControl::armTGT=15;
}

void motion12() {
  robotControl::armPIDenabled=false;
  robotControl::armSpeed=0;
  double point1[2] = {-6, 8};
  bez::scuffedPID(point1, 700, 1, 0, 0, 0, 0,false);
  robotControl::armPIDenabled=true;
  robotControl::armTGT=90;
  double** path = bez::generateStraightPath(point1, 700);
  double pow=0;
  double lastPow = 0;
  double spd = 0;
  double followDist =15;
  bez::resetPurePursuit();
  while(true) {
    if(robotControl::getArmPos()>75) {
      robotControl::armPIDenabled=false;
      robotControl::armSpeed=0;
    }
    conveyorSpd=600;
    if(bez::distanceTo(point1[0], point1[1])>35) {
      spd= 75;
    } else {
      spd= 40;
    }
    pow = slew(200, spd, lastPow);
    if((bez::purePursuit(path, followDist, pow, 15, false, false)==true)) {
      break; 
    }
    if(bez::distanceTo(point1[0], point1[1])<25) {
      break;
    }
    lastPow = pow;
    wait(20,msec);
  }
  double timer1 = Brain.Timer.time(msec);
  while(true) {
    spd=40;
    pow = slew(200, spd, lastPow);
    bez::setDrive_tank_abs(pow, 180);
    if(drive_range.objectDistance(mm)<200&&drive_range.objectDistance(mm)!=0&&drive_range.objectDistance(mm)>130) {
      break;
    }
    lastPow = pow;
    wait(20,msec);
  }
  printf("KUKLE: %lf", Brain.Timer.time(msec)-timer1);
  bez::setDrive_Tank_raw(0, 0);
  wait(200,msec); 
  conveyorSpd=600;
  bez::scuffedPID(90, 1300, 1, 0, 0, 0, 0);
  
}


double autonStartTime=0;
void park() {
  robotControl::driveBoost=true;
  //move platform down drive forward a bit
  double pitch_zero=inert.pitch();
  double parkTimer=Brain.Timer.time(msec);
  double pow=0;
  double lastPow = 00;
  double spd = 0;
  //safdhufdajoisfdajoisfdiojfsdaoisfda
  double point1[2] = {odom::x_position+25,odom::y_position};
  robotControl::armPIDenabled=false;
  robotControl::armSpeed=0;
  //initial settings
  double **path =bez::generateStraightPath(point1, 50);
  double followDist = 13;
  bez::resetPurePursuit();
  while(true) {
    spd=30;
    pow = slew(100, spd, lastPow);
    if((bez::purePursuit(path, followDist, pow, 5, true, false)==true)) {
      break; 
    }
    lastPow = pow;
    wait(20,msec);
    if(odom::x_position>6) {
      break;
    }
  }
  //fsdaosfdanafsdoiafsdln
  // rearMogoState=true;
  bez::setDrive_Tank_raw(0,0);
  robotControl::armPIDenabled=true;
  robotControl::armTGT= 10;
  robotControl::flippyPIDenabled=true;
  robotControl::flippyTGT=15;
  armAct.open();
  while(robotControl::getArmPos()>15) {
    wait(20,msec);
    bez::setDrive_tank_abs(0,90);
  }
  wait(400,msec);

  robotControl::armPIDenabled=false;
  robotControl::armSpeed=0;
  bez::setDrive_Tank_raw(7, 7);
  wait(200,msec);
  lastPow = 0;
  pow = 0;
  while((inert.pitch()-pitch_zero)>-15) {
    spd=40;
    pow = slew(75, spd, lastPow);
    lastPow = pow;
    wait(10,msec);
    bez::setDrive_tank_abs(pow,90);
  }
  conveyorSpd=0;

  double zeroDist=odom::x_position;
  //stage1
  double tgt = 33.1+odom::x_position;
  double timer1 = 999999;
  double diagTime = Brain.Timer.time(msec);
  int counter = 0;
  while(true) {
    if((abs(tgt-odom::x_position)<1.)&&timer1==999999) {
      timer1= Brain.Timer.time(msec);
    }
    // if(((Brain.Timer.time(msec)-timer1))>100) {
    //   break;
    // }
    spd = (tgt-odom::x_position)*4;
    if(spd>30) {
      spd=30;
    }
    if(spd<5) {
      spd=5;
      counter++;
    }
    if((inert.pitch()-pitch_zero)>-15) {
      break;
    }
    bez::setDrive_Tank_raw(spd, spd);
    lastPow = pow;
    wait(10,msec);
    printf("dist %lf\n", tgt-odom::x_position);
    printf("time %lf\n", Brain.Timer.time(msec)-diagTime);
    printf("pitch: %lf\n", inert.pitch());
    printf("spd: %lf\n", spd);
  }
  int counter2 = 0;
  double zero2 = odom::x_position;
  while(true) {
    counter2++;
    if((odom::x_position-zero2)<-1.4) {
      break;
    }
    bez::setDrive_Tank_raw(-40, -40);
    wait(10,msec);
  }
  printf("counter1: %d \n", counter);
  printf("counter2: %d \n", counter2);

  bez::setDrive_Tank_raw(0, 0);
  printf("ffos\n");
  printf("driveDist %lf\n", odom::x_position-zeroDist);
  printf("trueHeading: %lf \n", odom::trueHeading());
  printf("x_position: %lf\n", odom::x_position);
  printf("y_position: %lf\n", odom::y_position);
  printf("1234\n");
  printf("EXTRA TIME: %lf\n", Brain.Timer.time(msec)-autonStartTime);
  double Timer1  = Brain.Timer.time(msec);
  while((Brain.Timer.time(msec)-Timer1)<1400) {
    wait(20,msec);
    if((Brain.Timer.time(msec)<autonStartTime)>60000) {
      break;
    }
  }
  double ddd=odom::x_position;
  double timer3 = 0;
  while(true) {
    ddd=odom::x_position;
  if((inert.pitch()-pitch_zero)>5) {
      while((odom::x_position-ddd)>-1.) {
        bez::setDrive_Tank_raw(-50,-50);
        wait(20,msec);
      }
      bez::setDrive_Tank_raw(0, 0);
      wait(1000,msec);
  } else if((inert.pitch()-pitch_zero)<-5) {
      while((odom::x_position-ddd)<1.) {
        bez::setDrive_Tank_raw(50,50);
        wait(20,msec);
      }
      bez::setDrive_Tank_raw(0, 0);
      wait(1400,msec);
  } else {
     break;
  }
  wait(20,msec);
  }
  rearMogoState=true;
}

double inertialOffset=270;
void autonomous(void) {
  // ..........................................................................
  // Insert autonomous user code here.
  // ..........................................................................
  robotControl::mechMode=5;
  robotControl::mechTGT=5;
  odom::resetOdom();
  inert.setHeading(inertialOffset, degrees);
  armRot.resetPosition();
  flippy.resetPosition();
  task ape (background);
  wait(2,msec);
  task sus (flippyRunner);
  wait(2,msec);
  task sda (armRunner);
  wait(2,msec);
  task fdd (mechRunner);
  wait(2,msec);
  task sdd (odomRunner);
  wait(2,msec);
  task epe (diagnostic);
  wait(2,msec);
  task dsd (conveyorRunner);
  wait(2,msec);
  task sss (rightTrigger);
  wait(2,msec);
  task vis (visionRunner);
  wait(2,msec);
  colorMode=0;
  rearVis=true;

  




  //drive up to neutral and grab
  //also deployment sequence
  robotControl::driverControl=false;
  robotControl::volt_enabled=false;
  autonStartTime = Brain.Timer.time(msec);
  motion1();
  double timer1 = Brain.Timer.time(msec);
  printf("motion1: %lf\n", timer1-autonStartTime);
  motion2();
  double timer2 = Brain.Timer.time(msec);
  printf("motion2: %lf\n", timer2-timer1);
  motion3();
  double timer3 = Brain.Timer.time(msec);
  printf("motion3: %lf\n", timer3-timer2);
  motion4();
  double timer4 = Brain.Timer.time(msec);
  printf("motion4: %lf\n", timer4-timer3);
  motion5();
  double timer5 = Brain.Timer.time(msec);
  printf("motion5: %lf\n", timer5-timer4);
  motion6();
  double timer6 = Brain.Timer.time(msec);
  printf("motion6: %lf\n", timer6-timer5);
  motion7();
  double timer7 = Brain.Timer.time(msec);
  printf("motion7: %lf\n", timer7-timer6);
  motion8();
  double timer8 = Brain.Timer.time(msec);
  printf("motion8: %lf\n", timer8-timer7);
  motion9();
  double timer9 = Brain.Timer.time(msec);
  printf("motion9: %lf\n", timer9-timer8);
  motion10();
  double timer10 = Brain.Timer.time(msec);
  printf("motion10: %lf\n", timer10-timer9);
  motion11();
  double timer11 = Brain.Timer.time(msec);
  printf("motion11: %lf\n", timer11-timer10);
  motion12();
  double timer12 = Brain.Timer.time(msec);
  printf("motion12: %lf\n", timer12-timer11);
  park();

  bez::setDrive_Tank_raw(0, 0);
  printf("STOP TIME:%lf\n", (Brain.Timer.time(msec)-autonStartTime));
  printf("motion1: %lf\n", timer1-autonStartTime);
  printf("motion2: %lf\n", timer2-timer1);
  printf("motion3: %lf\n", timer3-timer2);
  printf("motion4: %lf\n", timer4-timer3);
  printf("motion5: %lf\n", timer5-timer4);
  printf("motion6: %lf\n", timer6-timer5);
  printf("motion7: %lf\n", timer7-timer6);
  printf("motion8: %lf\n", timer8-timer7);
  printf("motion9: %lf\n", timer9-timer8);
  printf("motion10: %lf\n", timer10-timer9);
  printf("motion11: %lf\n", timer11-timer10);
  printf("motion12: %lf\n", timer12-timer11);



  printf("trueHeading: %lf \n", odom::trueHeading());
  printf("x_position: %lf\n", odom::x_position);
  printf("y_position: %lf\n", odom::y_position);
  printf("1234\n");



}

int stupidDriveHelp() {
  while(true) {
    if(rearMogoState==true) {
      Controller1.rumble(".");
    }
    wait(1000,msec);
  }
  return 0;
}


void usercontrol(void) {
  // User control code here, inside the loop
  robotControl::driverControl=true;
  robotControl::mechTGT=2;

  inert.setHeading(inertialOffset, degrees);
  if(initialized==false) {
  flippy.resetPosition();
  armRot.resetPosition();
  task ape (background);
  wait(2,msec);
  task cock (diagnostic);
  wait(2,msec);
  task sus (flippyRunner);
  wait(2,msec);
  task sda (armRunner);
  wait(2,msec);
  task fdd (mechRunner);
  wait(2,msec);
  task sdd (odomRunner);
  wait(2, msec);
  task dum (stupidDriveHelp);
  wait(2,msec);
  task dsd (conveyorRunner);
  wait(2,msec);
  task sss (rightTrigger);
    wait(2,msec);
  task vis (visionRunner);

  initialized=true;
  printf("RESET:\n");
  }


  while (1) {
    if(rightTriggerMode==1) {
      //go up
      robotControl::manualArmSpeed=-100;
      robotControl::driveBoost=false;
    } else if(rightTriggerMode==2&&armBottom.pressing()==false) {
      // go down
      robotControl::manualArmSpeed=100;
      robotControl::driveBoost=false;
    } else {
      // neutral
      if (robotControl::driveBoost==true) {
        robotControl::manualArmSpeed=0;
      } else {
        if(armBottom.pressing()==false) {
          robotControl::manualArmSpeed=300;
        } else {
          robotControl::manualArmSpeed=200;
        }
      }
    }

    if(rightTriggerMode== 3) {
      robotControl::driveBoost=true;
    }


    if(Controller1.ButtonUp.pressing()==true&&robotControl::mechTGT==3) {
      //high
      robotControl::mechTGT = 4;
    } else if((Controller1.ButtonUp.pressing()==true&&robotControl::mechTGT==4)&&lastUp==false){
      //drop off high tower (mostly for skills)
      robotControl::driverControl=false;
      bez::setDrive_Tank_raw(60, 60);
      wait(200,msec);
      rearMogoState=true;
      wait(100,msec);
      robotControl::driverControl=true;
      robotControl::volt_enabled=true;
    }
      else if (Controller1.ButtonDown.pressing()==true) {
      //prep high
      robotControl::driveBoost=false;
      robotControl::mechTGT=3;
    } else if (Controller1.ButtonRight.pressing()==true) {
      //high
      robotControl::driveBoost=false;
      robotControl::mechTGT = 2;
    } else if (Controller1.ButtonLeft.pressing()==true) {
      //score front
      rearMogoState=false;
      robotControl::driveBoost=false;

      robotControl::driveBoost=false;
      robotControl::mechTGT = 0;
    }



    if (Controller1.ButtonL2.pressing()==true&&lastL2==false) {
      rearMogoState=!rearMogoState;
  
    }

    if (Controller1.ButtonL1.pressing()==true&&lastL1==false) {
      clawState=!clawState;
    }
    if(Controller1.ButtonB.pressing()==true) {
      robotControl::driveHold=!robotControl::driveHold;
    }

    if(Controller1.ButtonA.pressing()==true&&lastA==false) {
      conveyorSpd=-600;
    } else if(Controller1.ButtonA.pressing()==false&&lastA==true) {
      conveyorSpd=0;
    }
    if(Controller1.ButtonX.pressing()==true&&lastX==false) {
      if(conveyorSpd!=600) {
        conveyorSpd=600;
      } else {
        conveyorSpd=0;
      }
    }

    if(Controller1.ButtonY.pressing()==true) {

    //   robotControl::rezero_flippy=true;
    // } else if(Controller1.ButtonY.pressing()==false&&lastY==true) {
    //   robotControl::rezero_flippy=false;
    //   // printf("x_position: %lf\n", odom::x_position);
    //   // printf("y_position: %lf\n", odom::y_position);
    //   // printf("trueHeading: %lf \n", odom::trueHeading());
    //   // printf("encoderBack: %lf\n", encoder_back.position(degrees));
    //   // printf("------------------------------\n");
    //   robotControl::flippyPIDenabled=true;
    //   robotControl::flippyTGT=45;
      
    }

    
    // This is the main execution loop for the user control program.
    // Each time through the loop your program should update motor + servo
    // values based on feedback from the joysticks.

    // ........................................................................
    // Insert user code here. This is where you use the joystick values to
    // update your motors, etc.
    // ........................................................................
    lastButtons();
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
