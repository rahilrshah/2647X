#include "vex.h"
using namespace vex;

odom::odom(){
  printf("Odom Initialized\n");
}

void odom::resetOdom() {
  Ldrive1.resetPosition();
  Ldrive2.resetPosition();
  Rdrive1.resetPosition();
  Rdrive2.resetPosition();

  encoder_back.resetPosition();
  encoder_back_last=encoder_back.position(degrees);
  encoder_left_last = (Ldrive1.position(degrees)+Ldrive2.position(degrees))/2;
  encoder_right_last = (Rdrive1.position(degrees)+Rdrive2.position(degrees))/2;
  
  pitch_z = inert.pitch(degrees);
  roll_z = inert.roll(degrees);
  prevLocation = inert.heading();
  rotations = 0;
  x_position = 0;
  y_position = 0;
}

double odom::trueHeading() {
  if (prevLocation>200&&inert.heading()<160) {
    rotations++;
  }
  else if (prevLocation<160&&inert.heading()>200) {
    rotations--;
  }
  prevLocation = inert.heading();

  return (inert.heading() + 360*rotations);
}

void odom::updateOdom(){
  if((Brain.Timer.time(msec)-odomTimer)>21) {
    printf("bad shit happening\n");
    printf("refresh time: %lf\n", Brain.Timer.time(msec)-odomTimer);
  }
  double leftEncoder = (0.70588235)*((Ldrive1.position(degrees)+Ldrive2.position(degrees))/2);
  double rightEncoder = (0.70588235)*((Rdrive1.position(degrees)+Rdrive2.position(degrees))/2);
  odomAngle=((trueHeading()+odomAngle_last)/2)+inertialOffset;

  delta_y = (((leftEncoder-encoder_left_last) + (rightEncoder-encoder_right_last))/2)*degToDistance;
  delta_x  = (encoder_back.position(degrees)-encoder_back_last)*degToDistance;

  y_position += cos(odomAngle*degToRad)*delta_y;
  x_position += sin(odomAngle*degToRad)*delta_y;

  y_position += sin(odomAngle*degToRad)*delta_x;
  x_position += -cos(odomAngle*degToRad)*delta_x;

  odomAngle_last = trueHeading();
  encoder_back_last=encoder_back.position(degrees);
  encoder_left_last = leftEncoder;
  encoder_right_last = rightEncoder;
  odomTimer=Brain.Timer.time(msec);
}
