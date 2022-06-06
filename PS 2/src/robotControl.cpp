#include "vex.h"
using namespace vex;


robotControl::robotControl(){
  printf("test1\n");

}

void enableDrive() {
  robotControl::driverControl=true;
  robotControl::volt_enabled=true;
  
}
void robotControl::controlPnem() {
  if (shiftMode==0) {
    driveShift.close();
    mechShift.close();
  } else if (shiftMode == 1) {
    driveShift.open();
    mechShift.close();
  } else if (shiftMode == 2) {
    driveShift.open();
    mechShift.open();
  } else if (shiftMode == 3) {
    driveShift.close();
    mechShift.open();
  }
}


double scale(int input, int t) {
  int exponent = (fabs(input)-100)*t*0.001;
  return pow(2.71, exponent)*input;
}

void robotControl::controlShift() {
  if(armSpeed==300&&lastArmSpeed!=300) {
    armHoldPosL = Ldrive3_arm.position(degrees);
    armHoldPosR = Ldrive3_arm.position(degrees);
    printf("reg\n");
  }



  if (driverControl == true) {
    //     leftDrive = Controller1.Axis3.position();
    // rightDrive = Controller1.Axis2.position();
    //TODO: KRISH CHANGE
    leftDrive = ::scale(Controller1.Axis3.position(),15)*.9;
    // printf("scale:%lf\n",::scale(Controller1.Axis3.position(),15) );
    rightDrive = ::scale(Controller1.Axis2.position(),15)*.9;
  }

  double lVolt = leftDrive*140;
  double rVolt = rightDrive*140;

    if(driverControl==true) {
    if(driveHold==false) {
      if(volt_enabled==true) {
        Ldrive1.spin(fwd, lVolt, voltageUnits::mV);
        Ldrive2.spin(fwd, lVolt, voltageUnits::mV);
        Rdrive1.spin(fwd, rVolt, voltageUnits::mV);
        Rdrive2.spin(fwd, rVolt, voltageUnits::mV);
      } else {
        Ldrive1.spin(fwd, leftDrive*6, rpm);
        Ldrive2.spin(fwd, leftDrive*6, rpm);
        Rdrive1.spin(fwd, rightDrive*6, rpm);
        Rdrive2.spin(fwd, rightDrive*6, rpm);
      }

    } else {
      //if threre is no input in axis hold motors
      if(leftDrive==0) {
        Ldrive1.stop(hold);
        Ldrive2.stop(hold);
      } else {
        if(volt_enabled==true) {
          Ldrive1.spin(fwd, lVolt, voltageUnits::mV);
          Ldrive2.spin(fwd, lVolt, voltageUnits::mV);
        } else {
          Ldrive1.spin(fwd, leftDrive*6, rpm); 
          Ldrive2.spin(fwd, leftDrive*6, rpm);
        }

      }
      if(rightDrive==0) {
        Rdrive1.stop(hold);
        Rdrive2.stop(hold);
      } else {
        if(volt_enabled==true) {
          Rdrive1.spin(fwd, rVolt, voltageUnits::mV);
          Rdrive2.spin(fwd, rVolt, voltageUnits::mV);
        } else {
          Rdrive1.spin(fwd, rightDrive*6, rpm);
          Rdrive2.spin(fwd, rightDrive*6, rpm);
        }

      }
    }

    } else {
      if(leftDrive==0) {
        Ldrive1.stop(hold);
        Ldrive2.stop(hold);
      } else {
        if(volt_enabled==true) {
          Ldrive1.spin(fwd, lVolt, voltageUnits::mV);
          Ldrive2.spin(fwd, lVolt, voltageUnits::mV);
        } else {
          Ldrive1.spin(fwd, leftDrive*6, rpm);
          Ldrive2.spin(fwd, leftDrive*6, rpm);
        }

      }
      if(rightDrive==0) {
        Rdrive1.stop(hold);
        Rdrive2.stop(hold);
      } else {
        if(volt_enabled==true) {
          Rdrive1.spin(fwd, rVolt, voltageUnits::mV);
          Rdrive2.spin(fwd, rVolt, voltageUnits::mV);
        } else {
          Rdrive1.spin(fwd, rightDrive*6, rpm);
          Rdrive2.spin(fwd, rightDrive*6, rpm);
        }
      }
    }
  

  if ((leftDrive!=0&&rightDrive!=0)&&(armSpeed==0)&&driveBoost==true) {
    //engage driveTrain
    transmissionState=0;
  } 

  if(armSpeed!=0) {
    //engage mogo
    transmissionState=1;
  }

  if(transmissionState!=transmissionState_last) {
    shifting=true;
    shiftTarget = transmissionState;
    shiftTimer = Brain.Timer.time(msec);

  }
  
  if (shifting==true) {
    if((Brain.Timer.time()-shiftTimer)<100) {
      //put them into neutral
      shiftMode = 1;

    } else if ((Brain.Timer.time(msec)-shiftTimer)<200){ 
      //sync the motors
      driveEngaged=false;
      armEngaged=false;

      if(transmissionState==0) {
        if(volt_enabled==true) {
          Ldrive3_arm.spin(fwd, lVolt, voltageUnits::mV);
          Rdrive3_arm.spin(fwd, rVolt, voltageUnits::mV);
        } else {
          Ldrive3_arm.spin(fwd, leftDrive*6, rpm);
          Rdrive3_arm.spin(fwd, rightDrive*6, rpm);
        }

      } else if (transmissionState==1) {
        Ldrive3_arm.spin(fwd, armSpeed*6, rpm);
        Rdrive3_arm.spin(fwd, armSpeed*6, rpm);
      }

    } else if ((Brain.Timer.time(msec)-shiftTimer)<300) {
      //execute the shift
      if(transmissionState==0) {
        if(volt_enabled==true) {
          Ldrive3_arm.spin(fwd, lVolt, voltageUnits::mV);
          Rdrive3_arm.spin(fwd, rVolt, voltageUnits::mV);
        } else {
          Ldrive3_arm.spin(fwd, leftDrive*6, rpm);
          Rdrive3_arm.spin(fwd, rightDrive*6, rpm);
        }
      } else if (transmissionState==1) {
        Ldrive3_arm.spin(fwd, armSpeed*6, rpm);
        Rdrive3_arm.spin(fwd, armSpeed*6, rpm);
      } 
      if(transmissionState==0) {
        shiftMode = 0;
      } else if(transmissionState==1) {
        shiftMode = 2;
      }
    } else {
      shifting = false;
      if (transmissionState==0) {
        driveEngaged=true;
      } else if(transmissionState==1) {
        armEngaged=true;
      }
    }
 }
      

  if(driveEngaged==true) {
      Ldrive3_arm.setMaxTorque(100, percent);
      Rdrive3_arm.setMaxTorque(100, percent);
      Ldrive3_arm.setBrake(coast);
      Rdrive3_arm.setBrake(coast);
      if(volt_enabled==true) {
        Ldrive3_arm.spin(fwd, lVolt, voltageUnits::mV);
        Rdrive3_arm.spin(fwd, rVolt, voltageUnits::mV);
      } else {
        Ldrive3_arm.spin(fwd, leftDrive*6, rpm);
        Rdrive3_arm.spin(fwd, rightDrive*6, rpm);
      }
  }
  
  if(armEngaged ==true) {

    if(armSpeed==300) {
      Ldrive3_arm.setMaxTorque(armTorque, percent);
      Rdrive3_arm.setMaxTorque(armTorque, percent);
      Ldrive3_arm.stop(hold);
      Rdrive3_arm.stop(hold);
      // Ldrive3_arm.startSpinTo(armHoldPosL, degrees, 100, rpm);
      // Rdrive3_arm.startSpinTo(armHoldPosR, degrees, 100, rpm);
    } else if (armSpeed == 200) {
      Ldrive3_arm.setMaxTorque(armTorque, percent);
      Rdrive3_arm.setMaxTorque(armTorque, percent);
      Ldrive3_arm.stop(coast);
      Rdrive3_arm.stop(coast);
    } else {
      Ldrive3_arm.setMaxTorque(armTorque, percent);
      Rdrive3_arm.setMaxTorque(armTorque, percent);
      Ldrive3_arm.spin(fwd, armSpeed*6, rpm);
      Rdrive3_arm.spin(fwd, armSpeed*6, rpm);
    }
  }
  lastArmSpeed=armSpeed;
  transmissionState_last = transmissionState;
  controlPnem();
}



double robotControl::getFlippyPos() {
  double maxPos=1080;
  return flippy.position(degrees)/maxPos*100;

}

double robotControl::flippyPID() {
  double kp = 14;
  double ki = 0;
  double kd = 75;
  double currPos = 0;
  double maxPos = 1080;
  double integral = 0;
  double error = 0;
  double lastError = 0;
  double lastRefresh = 0;

  while(true) {

    currPos = flippy.position(degrees)/maxPos*100;
    error = currPos-flippyTGT;
    double output = kp*error + kd*(error-lastError)/(20);

    if(rezero_flippy==true) {
      while(rezero_flippy==true) {
        flippy.spin(forward, -7000, voltageUnits::mV);
        wait(20,msec);
      }
      flippy.setPosition(0, degrees);
    } else if(flippyPIDenabled==true) {
      if(flippyTGT==200) {
        flippy.stop(coast);
      } else if(flippyTGT==300){
        flippy.stop(hold);
      } else if(flippyTGT==400) {
        flippy.spin(forward, -1500, voltageUnits::mV);
      } else {
        if(fabs(output)>flippyPid_speed) {
          if(output>0) {
            output=flippyPid_speed;
          } else {
            output=-flippyPid_speed;
          }
        }

        flippy.spin(forward,-output,rpm);
      }
    }

    lastError=error;
    lastRefresh = Brain.Timer.time(msec);  
    wait(20,msec);

  }
  return 0;
}


double robotControl::armPID() {
  double kp = 3.5;
  double ki = 0;
  double kd = 5;
  double currPos = 0;
  double maxPos = 500;
  double integral = 0;
  double error = 0;
  double lastError = 0;
  double lastRefresh = 0;
  double diagTimer = 0;

  while(true) {
    currPos = armRot.position(degrees)/maxPos*100;
    error = currPos-armTGT;
    double output = kp*error + kd*(error-lastError);
    if(armBottom.pressing()==true&&currPos<30) {
      armRot.setPosition(0, degrees);
    }
    if (armPIDenabled==true) {
      if(armTGT==400) {
        armSpeed=0;
      } else if(armTGT==200) {
        armSpeed=200;
      } else {
        if(armBottom.pressing()==true&&output>0) {
          armSpeed=200;
        }

        if(fabs(output)>armPid_speed) {
          if(output>0) {
            armSpeed=armPid_speed;
          } else {
            armSpeed=-armPid_speed;
          }
        } else {
          armSpeed=output;
        }
        if(armSpeed==0) {
          armSpeed=-1;
        }
      }
      
    }

    // if((Brain.Timer.time(msec)-diagTimer)>100) {
    //   printf("err: %lf \n", error);
    //   printf("dval: %lf\n", kd*(error-lastError));
    //   printf("output: %lf\n", output);
    //   printf("------------------------------\n");

    //   diagTimer=Brain.Timer.time(msec);
    // }
    lastError=error;
    lastRefresh = Brain.Timer.time(msec);

    wait(20,msec);
  }
  return 0;

}


double robotControl::getArmPos() {
  double maxPos=500;
  return armRot.position(degrees)/maxPos*100;

}


void robotControl::mechControl() {
  bool shiftReady=false;
  int mechModeLast=0;
  bool peginPos=false;
  double lastArmPos=0;
  double initialTimer=0;
  int highProg=0;
  while(true) {
  if(mechTGT!=mechMode&&shiftReady==true) {
        printf("apesdfsd\n");
        printf("mechMode: %d\n", mechMode);
        printf("mechTGT: %d\n", mechTGT);
    if(mechTGT==0) {
      if(mechMode==2||mechMode==3||mechMode==4||mechMode==5) {
        mechMode = 1;
      }else if(mechMode==1) {
        mechMode=0;
      }
    } else if(mechTGT==1) {
      mechMode=1;
    } else if(mechTGT==2||mechTGT==3||mechTGT==4||mechTGT==5) {
      if(mechMode==0) {
        mechMode=2;
      } else {
        mechMode=mechTGT;
      }
    }
  }
  //handles actual motor movements;
  if(mechMode!=mechModeLast) {
    shiftReady=false;
    peginPos=false;
    highProg=0;
    initialTimer=999999999999999999;
    
    printf("ape\n");
  }

  if(mechMode==0) {
    armAct.close();
    armPid_speed=100;
    flippyPid_speed=100;

    armPIDenabled=false;
        //peg loading----------------------------------------------------------------------------------------------------------------------------
    if(getArmPos()>15&&lastArmPos<15) {
      initialTimer=Brain.Timer.time(msec);
    }
    if((Brain.Timer.time(msec)-initialTimer)>300|pegIn.pressing()==true) {
      peginPos=true;
    }
    if(peginPos==false) {
      armSpeed=-60;
    } else {
      shiftReady=true;
    }
    if(shiftReady==true) {
      if(initialTimer>1000) {
        flippy.resetPosition();
      }
      flippyTGT = 400;
      if(driveBoost!=true) {
        armSpeed=200;
      } else {
        armSpeed=0;
      }
    }
  } else if(mechMode==1) {
    armPid_speed=100;
    flippyPid_speed=100;

    //arm fully down peg fully down------------------------------------------------------------------------------------------------------------
    //enter state 0
    peg.close();
    if(driverControl==false) {
      armAct.open();
    } else {
      armAct.close();
    }

    if (shiftReady==false) {
      if(getArmPos()<5) {
        if(getFlippyPos()>5) {
          flippyPIDenabled=true;
          flippyTGT=0;
        } else {
          flippyPIDenabled=true;
          flippyTGT=200;
        }
      } else {
          flippyPIDenabled=true;
          flippyTGT=15;
      }
    
      if(getArmPos()>5) {
        armPIDenabled=true;
        armTGT=0;
      } else {
        armPIDenabled=false;
        if(armBottom.pressing()!=true) {
          armSpeed=10;
        } else {
          armSpeed=200;
        }
      }
    } else {
      armPIDenabled=false;
      armSpeed=200;
    }
    if(getFlippyPos()<5&&getArmPos()<5) {
      shiftReady=true;
    }
  } else if(mechMode==2) {
    armPid_speed=100;
    flippyPid_speed=60;

        armAct.close();

    //to get out of state 0, also neutral state with manual arm control control;
    //standard match peg position
    if(shiftReady==false) {
      flippyPIDenabled=true;
      flippyTGT=30;
      armPIDenabled=false;
      if(armBottom.pressing()==false) {
        armSpeed=35;
      } else {
        armSpeed=0;
      }

      if(getFlippyPos()>20) {
        shiftReady=true;
        armPIDenabled=false;
        armSpeed=manualArmSpeed;
        flippyPIDenabled=true;
        flippyTGT=35;
        peg.open();
      }
    } else {
        shiftReady=true;
        armPIDenabled=false;
        armSpeed=manualArmSpeed;
        flippyPIDenabled=true;
        flippyTGT=35;
        peg.open();
    }
  } else if(mechMode==3) {
    armPid_speed=100;
    flippyPid_speed=100;
    armPIDenabled=false;
    armSpeed=manualArmSpeed;
    //prep to score high--------------------------------------------------------------
    flippyPIDenabled=true;
    flippyTGT=60;
    shiftReady=true;
    armAct.open();

  } else if(mechMode==4) {
    double flippy1=40;
    double arm1=118;
    double flippy2=31;
    double arm2=87;

    //score high--------------------------------------------------------------------------------------
    if(shiftReady==false) {
    if(highProg==0) {
      if(driveHold==false) {
        armPid_speed=100;
        flippyPid_speed=100;
      } else {
        armPid_speed=40;
        flippyPid_speed=40;
      }
      flippyPIDenabled=true;
      flippyTGT=60;
      armAct.open();
      armPIDenabled=true;
      armTGT=arm1;
      if(getArmPos()>arm1-10) {
        highProg++;
        wait(100,msec);
      }
    } else if(highProg==1) {
      if(driveHold==false) {
        armPid_speed=100;
        flippyPid_speed=75;
      } else {
        armPid_speed=40;
        flippyPid_speed=40;
      }

      flippyPIDenabled=true;
      flippyTGT=flippy1;
      armAct.open();
      armPIDenabled=true;
      armTGT=arm1;
      if(getFlippyPos()<flippy1+5) {
        highProg++;
        wait(100,msec);
      }
    } else if(highProg==2) {
      if(driveHold==false) {
        armPid_speed=60;
        flippyPid_speed=10;
      } else {
        armPid_speed=25;
        flippyPid_speed=10;
      }
      flippyPIDenabled=true;
      flippyTGT=flippy2;
      armAct.open();
      armPIDenabled=true;
      armTGT=arm2;
      if(getArmPos()<arm2+5) {
        highProg++;
      }
    } else if(highProg==3) {
      if(driveHold==false) {
        armPid_speed=60;
        flippyPid_speed=35;
      } else {
        armPid_speed=25;
        flippyPid_speed=35;
      }

      armAct.open();
      flippyPIDenabled=true;
      flippyTGT=flippy2;
      armPIDenabled=true;
      armTGT=arm2;
      if(getFlippyPos()<flippy2+2) {
        highProg++;
        wait(100,msec);
        shiftReady=false;
      }
    } else if(highProg==4) {


        armPid_speed=25;
        flippyPid_speed=25;
        flippyTGT=flippy2;
        peg.close();
        armTGT=arm2;

        wait(500,msec);
        armTGT=arm2-2;
        flippyTGT=flippy2+2;
        wait(400,msec);
        highProg++;
      } else if(highProg==5) {
        peg.close();
        armPid_speed=100;
        flippyPid_speed=50;

        flippyPIDenabled=true;
        flippyTGT=flippy2+10;
        wait(50,msec);
        armPIDenabled=true;
        armTGT=arm2-15;
        wait(200,msec);
        shiftReady=true;
      }
    }
    
      if(shiftReady==true) {
        printf("ape\n");
        peg.close();
        armPid_speed=100;
        flippyPid_speed=50;

        flippyPIDenabled=true;
        flippyTGT=50;
        if(driverControl==true) {
          armPIDenabled=false;
          armSpeed=manualArmSpeed;
        } else {
          armPIDenabled=true;
          armTGT=arm2-15;
        }

      }
    



  } else if(mechMode==5) {
    shiftReady=true;
    //sussy mode for auton (mega sus);
    //arm position and flippy posiiton are controlled completely manually
    //do whatever the fuck you want ig
  }
  lastArmPos=getArmPos();
  mechModeLast=mechMode;
  wait(20,msec);
  }
}


//find shotAngle with bools and pneum.
double turrPrevError = 0;
void robotControl::turretAim(double target[], double x_speed, double y_speed){
  if(bez::distanceTo(target[0], target[1])>robotControl::changedist){
    robotControl::highShot=true;
    robotControl::shotAngle=90;
  } else{
    robotControl::highShot=false;
    robotControl::shotAngle=0;
  }

  if(robotControl::highShot){
    robotControl::shotAngle=90;
    robotControl::shotHeight=10;
  } else{
    robotControl::shotAngle=0;
    robotControl::shotHeight=0;
  }
  double turrAngle = turret.position(degrees) + odom::trueHeading();
  double turrAirTime = sqrt(((bez::distanceTo(target[0], target[1])*tan(robotControl::shotAngle)-robotControl::shotHeight)*robotControl::turrOffConst-robotControl::deltaheight)/4.9);
  double turretOffset = (x_speed*cos(bez::angleTo(target[0], target[1]))+y_speed*sin(bez::angleTo(target[0], target[1])))*turrAirTime*robotControl::offsetConstant;
  double turrTgt = bez::angleTo(target[0], target[1])+turretOffset;
  robotControl::flySpeed = (bez::distanceTo(target[0],target[1])/(turrAirTime*cos(robotControl::shotAngle)))*robotControl::flyConst;
  robotControl::turretSpeed = robotControl::turretKp*(turrTgt-turrAngle)+((turrTgt - turrAngle) - turrPrevError)*robotControl::turretKd;
  turrPrevError = turrTgt-turrAngle;
}



