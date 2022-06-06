#include "vex.h"
using namespace vex;

bez::bez(){
  printf("test3\n");
}
double * bez::interpolateBtw(double p1[], double p2[], double progress) {
    double* output = 0;
    output = new double[2];
    
    output[0] = p2[0]*(progress) + p1[0]*(1 - progress);
    output[1] = p2[1]*(progress) + p1[1]*(1 - progress);
    return output;
}

double * bez::goalAdj(double goalTGT[], double offset) {
  double* currentPos = 0;
  currentPos = new double[2];
  currentPos[0]=odom::x_position;
  currentPos[1]=odom::y_position;
  double dist = bez::distanceTo(goalTGT[0], goalTGT[1]);
  double progress = ((dist-offset)/dist);
  return bez::interpolateBtw(currentPos,goalTGT, progress);
}


double * bez::quadBezierCalc(double startPoint[], double controlPoint[], double endPoint[], double progress){

    double* output = 0;
    output = bez::interpolateBtw(bez::interpolateBtw(startPoint, controlPoint, progress), bez::interpolateBtw(controlPoint, endPoint, progress), progress);

  return output;
}

double ** bez::BezierArray(double startPoint[], double controlPoint[], double endPoint[], int size) {
    double** output = 0;
    output = new double*[size+1];
    
    double stepSize = (double) 1/(size-1);
    double progress = 0;
    
    output[0] = new double[2];
    output[0][0]  = size;
    
    for (int i = 1; i<size+1; i++) {
        
        output[i] = quadBezierCalc(startPoint, controlPoint, endPoint, progress);
        progress += stepSize;
    }
    return output;
    
}

double ** bez::generateStraightPath(double *target, int size) {
    double** output = 0;
    output = new double*[size+1];
    
    double stepSize = (double) 1/(size-1);
    double progress = 0;

    double* currentPos = 0;
    currentPos = new double[2];
    currentPos[0]=odom::x_position;
    currentPos[1]=odom::y_position;

    output[0] = new double[2];
    output[0][0]  = size;
    for (int i = 1; i<size+1; i++) {
        
        output[i] = interpolateBtw(currentPos, target, progress);
        progress += stepSize;
    }
    return output;
}


double ** bez::addArrays(double ** array1, double ** array2) {
  double** output = 0;
  int size = array1[0][0]+array2[0][0];
  output = new double*[size+1];

  output[0] = new double[2];
  output[0][0]  = size;

  int counter = 1;
  for(int i=1; i<=array1[0][0]; i++) {
    output[counter] = new double[2];
    output[counter][0] = array1[i][0];
     output[counter][1] = array1[i][1];
     counter++;
  }

  for(int i=1; i<=array2[0][0]; i++) {
    output[counter] = new double[2];
    output[counter][0] = array2[i][0];
     output[counter][1] = array2[i][1];
     counter++;
  }

  return output;
}

double bez::distanceTo(double x, double y) {
  return sqrt(pow((x-odom::x_position),2) + pow((y-odom::y_position),2));
}

double bez::distanceTo(double x, double y, double thetaOffset, double distOffset){
  double currx = odom::x_position+distOffset*sin(odom::trueHeading()*degToRad+thetaOffset*degToRad);
  double curry = odom::y_position+distOffset*cos(odom::trueHeading()*degToRad+thetaOffset*degToRad);
  return sqrt(pow((x-currx),2) + pow((y-curry),2));
}

double bez::angleTo(double x, double y){
  return atan((x-odom::x_position)/(y-odom::y_position));
}

double bez::angleTo(double x, double y, double thetaOffset, double distOffset){
  double currx = odom::x_position+distOffset*sin(odom::trueHeading()*degToRad+thetaOffset*degToRad);
  double curry = odom::y_position+distOffset*cos(odom::trueHeading()*degToRad+thetaOffset*degToRad);
  double xdist = x-currx;
  double ydist = y-curry;
  return atan(xdist/ydist);
}

double bez::goToPoint(double x, double y) {
  double x_delta = x - odom::x_position;
  double y_delta = y - odom::y_position;

  double angleOut = atan(x_delta/y_delta)/degToRad;
  if (y_delta<0) {
    angleOut += 180;
  }
  return angleOut;
}

double bez::goToPoint(double x1, double y1, double x2, double y2) {
  double x_delta = x1 - x2;
  double y_delta = y1 - y2;

  double angleOut = atan(x_delta/y_delta)/degToRad;
  if (y_delta<0) {
    angleOut += 180;
  }
  return angleOut;
}


void bez::setDrive_tank(double strPow, double turnPow) {
  double l_pow = strPow+turnPow;
  double r_pow = strPow-turnPow;
  //if left is reading more than 100% power, decrease the right side power proportionally
  if (l_pow>100) {
    r_pow = r_pow - (l_pow-100);
  } else if (r_pow>100) {
    l_pow = l_pow - (r_pow-100);
  }
 robotControl::leftDrive=l_pow;
 robotControl::rightDrive=r_pow; 
}

void bez::setDrive_tank_abs(double strPow, double tgtAngle) {
  double error = fmod(odom::trueHeading(),360)-fmod(tgtAngle,360);
  while (error>180||error<-180) {
    if (error>180) {
      error=error-360;
    } else {
      error=error+360;
    }
  }
  double output = error*bez::setDrive_tank_abs_kP;
  setDrive_tank(strPow, -output);

}

void bez::setDrive_Tank_raw(double l_pow, double r_pow) {
  robotControl::leftDrive=l_pow;
  robotControl::rightDrive=r_pow;
}

void bez::resetPurePursuit() {
  purePursuit_final = false;
  purePursuit_point = 1;
  purePursuit_initialTurn = false;
}
bool bez::purePursuit(double** path, double pursuitDistance, double speed, double brakeDist, bool brake,bool reverse) {

  int size = path[0][0];

  //sets the target point to the first point in the array if starting profile
    
  //If the robot is within the pursuitDist of the Point its currently targeting, move to the next point in the array
  while(bez::distanceTo(path[bez::purePursuit_point][0],path[bez::purePursuit_point][1])<pursuitDistance&&bez::purePursuit_final == false) {
    if (bez::purePursuit_point >= size) {
      bez::purePursuit_final = true;
    } else {
      bez::purePursuit_point++;
      bez::purePursuitTarget_x = path[bez::purePursuit_point][0];
      bez::purePursuitTarget_y = path[bez::purePursuit_point][1];
      bez::point_Dist = sqrt(pow((path[bez::purePursuit_point][0]-path[bez::purePursuit_point-1][0]),2) + pow((path[bez::purePursuit_point][0]-path[bez::purePursuit_point-1][0]),2));
      bez::purePursuit_pathDist = 0;
    }

  }

  if (fabs(bez::goToPoint(bez::purePursuitTarget_x, bez::purePursuitTarget_y)-odom::trueHeading())<bez::angTol) {
    bez::purePursuit_initialTurn = false;
  }

  if (bez::purePursuit_initialTurn == true) {
  } else {


  //Moves the pursuit point farther along if the robot is within a certain distance
    if (bez::distanceTo(bez::purePursuitTarget_x, bez::purePursuitTarget_y)<pursuitDistance) {
      if (bez::purePursuit_final !=true) {
        double moveDist = pursuitDistance - bez::distanceTo(bez::purePursuitTarget_x, bez::purePursuitTarget_y);
        bez::purePursuit_pathDist += moveDist;
        double pathProgress = bez::purePursuit_pathDist / bez::point_Dist;
        bez::purePursuitTarget_x = path[bez::purePursuit_point][0]*(pathProgress) + path[bez::purePursuit_point-1][0]*(1-pathProgress);
        bez::purePursuitTarget_y = path[bez::purePursuit_point][1]*(pathProgress) + path[bez::purePursuit_point-1][1]*(1-pathProgress);
      } else {
        bez::purePursuitTarget_x = path[bez::purePursuit_point][0];
        bez::purePursuitTarget_y = path[bez::purePursuit_point][1];
      }

      // printf("Target_x: %lf\n", purePursuitTarget_x);
      // printf("Target_y: %lf\n", purePursuitTarget_y);
      // printf("Pos_x: %lf\n", x_position);
      // printf("Pos_y: %lf\n", y_position);
      // printf("DIST: %lf\n", distanceTo(purePursuitTarget_x, purePursuitTarget_y));
      // printf("purePursuit_point: %d\n", purePursuit_point);
      // printf("--------------------------------------------------------------\n");


    } 

  //Moves the robot towards the current pursuit point
  double mod =0;
  if(reverse==true) {
    speed = -1*speed;
    mod=180;
  }
    double error = bez::distanceTo(path[size][0], path[size][1]);

    double output = sqrt(error)*(speed/sqrt(brakeDist));

    if (fabs(output)>fabs(speed) || brake==false) {
      bez::setDrive_tank_abs(speed,mod+ bez::goToPoint(bez::purePursuitTarget_x, bez::purePursuitTarget_y));
    } else {
      if(error>pursuitDistance) {
        bez::setDrive_tank_abs(output,mod+ bez::goToPoint(bez::purePursuitTarget_x, bez::purePursuitTarget_y));
      } else {
        bez::setDrive_tank_abs(output,mod+ bez::goToPoint(path[bez::purePursuit_point][0], path[bez::purePursuit_point][1], path[bez::purePursuit_point-1][0], path[bez::purePursuit_point-1][1]));
      }
    }


  
    if (((bez::purePursuit_point >= size)&&(error-bez::purePursuit_error_last)>0)&&error<6) {
      return true;
    }


    bez::purePursuit_error_last= error;
  }

    

  return false; 
}

void bez::scuffedPID(int orient, int time1,double Kp, double Ki, double Kd, double Ioffzone,double accError) {
  PIDstartTime=Brain.Timer.time(msec);
  turnIntegral=0;

  while(time1>Brain.Timer.time(msec)-PIDstartTime) {
    double PIDerror = odom::trueHeading()-orient;
    while(fabs(PIDerror)>180) {
      if(PIDerror>180) {
        PIDerror-=360;
      } else {
        PIDerror+=360;
      }
    }

    if(fabs(PIDerror)<Ioffzone) 
      turnIntegral += PIDerror;
    else{
      turnIntegral = 0;
    }
    double turnD=Perror-PIDerror;
    double turnPow = int(PIDerror*Kp+turnIntegral*Ki+turnD*Kd);
    if (turnPow<80) {
      bez::setDrive_Tank_raw(-turnPow,turnPow);
    } else {
      bez::setDrive_Tank_raw(-80,80);
    }
    wait(20, msec);

  }
  double headingDiff = odom::trueHeading();
  bez::setDrive_Tank_raw(0,0);
  printf("headingDiff %lf\n",odom::trueHeading()-headingDiff);
  wait(100,msec);
}
void bez::scuffedPID(double target[], int time1,double Kp, double Ki, double Kd, double Ioffzone,double accError, bool reversed) {
  PIDstartTime=Brain.Timer.time(msec);
  turnIntegral=0;


  while(time1>Brain.Timer.time(msec)-PIDstartTime) {
    double orient = goToPoint(target[0], target[1]);
    if(reversed==true) {
      orient +=180;
    }
    double PIDerror = odom::trueHeading()-orient;
    while(fabs(PIDerror)>180) {
      if(PIDerror>180) {
        PIDerror-=360;
      } else {
        PIDerror+=360;
      }
    }

    if(fabs(PIDerror)<Ioffzone) 
      turnIntegral += PIDerror;
    else{
      turnIntegral = 0;
    }
    double turnD=Perror-PIDerror;
    double turnPow = int(PIDerror*Kp+turnIntegral*Ki+turnD*Kd);

    if (turnPow<80) {
      bez::setDrive_Tank_raw(-turnPow,turnPow);
    } else {
      bez::setDrive_Tank_raw(-80,80);
    }

    wait(20, msec);

  }
  double headingDiff = odom::trueHeading();
  bez::setDrive_Tank_raw(0,0);
  printf("headingDiff %lf\n",odom::trueHeading()-headingDiff);
  wait(100,msec);
}