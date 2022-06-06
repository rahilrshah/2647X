using namespace vex;

#ifndef bez_h
#define bez_h

class bez{
  private:
    static double PIDstartTime;
    static double turnIntegral;
    static double Perror;
    
  public:   
    bez();
    static double angTol;
    static double purePursuitTarget_x;
    static double purePursuitTarget_y;

    static int purePursuit_point;
    static double purePursuit_pathDist;
    static double point_Dist;
    static bool purePursuit_final;
    static double purePursuit_error_last;
    static bool purePursuit_initialTurn;

    constexpr const static double degToRad = 0.01745329;
    constexpr const static double setDrive_tank_abs_kP = 1.1;


    static double * interpolateBtw(double p1[], double p2[], double progress);
    static double * quadBezierCalc(double startPoint[], double controlPoint[], double endPoint[], double progress);
    static double ** BezierArray(double startPoint[], double controlPoint[], double endPoint[], int size);
    static double ** addArrays(double ** array1, double ** array2);
    static double **generateStraightPath(double target[], int size);
    static double distanceTo(double x, double y);
    static double distanceTo(double x, double y, double thetaOffset, double distOffset);
    static double angleTo(double x, double y);
    static double angleTo(double x, double y, double thetaOffset, double distOffset);
    static double goToPoint(double x, double y);
    static double straightPath(double x, double y);
    static double goToPoint(double x1, double y1, double x2, double y2);
    static void setDrive_Tank_raw(double l_pow, double r_pow);
    static void setDrive_tank(double strPow, double turnPow);
    static void setDrive_tank_abs(double strPow, double tgtAngle);
    static void resetPurePursuit();
    static bool purePursuit(double** path, double pursuitDistance, double speed, double brakeDist, bool brake, bool reversed);
    static void scuffedPID(int orient, int time1,double Kp, double Ki, double Kd, double Ioffzone,double accError);
    static void scuffedPID(double target[], int time1,double Kp, double Ki, double Kd, double Ioffzone,double accError, bool reversed);
    static double * goalAdj(double goalTGT[], double offset);

};

#endif