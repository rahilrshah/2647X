using namespace vex;

#ifndef odom_h
#define odom_h

class odom{
  private:
    // used in the trueHeading method to give a winding heading that takes into account gyro drift
    static int prevLocation;
    static int rotations;
    // initial zero values of the pitch and roll
    static double pitch_z;
    static double roll_z;
    //conversion factors
    constexpr static const double left_rightDiffToDeg = 10.61111111111; 
    constexpr static const double strafeTurnOffset = 4.42;
    constexpr static const double degToRad = 0.01745329;
    constexpr static const double inertialOffset=0.40;

    
    constexpr static const double degToDistance = 0.0242261104;
    constexpr static const double strafeAngleCorr = 0.00214126;

    //the angle that the odom uses to calculate position change
    static double odomAngle;
    static double odomAngle_last;

    //record the previus encoder values
    static double encoder_left_last;
    static double encoder_right_last;
    static double encoder_back_last;
    static double odomTimer;
    static double delta_x;
    static double delta_y;

  public: 

    static double x_position;
    static double y_position;

    odom();
    static double trueHeading();
    static void updateOdom();
    static void resetOdom();
    
    
};

#endif