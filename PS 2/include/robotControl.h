using namespace vex;

#ifndef robotControl_h
#define robotControl_h

class robotControl{
  private:
  

  //holds the position of the two shift gears
  static bool driveEngaged;
  static bool armEngaged;

  //is true when the robot is in an active shift
  static bool shifting;

  static double shiftTimer;
  //holds the final desired shifting state (prevents sus stuff)
  static int shiftTarget;

  //represents the intention of the shifting system, what it wants the final state to be once it is done, not the actual current state
  //0-engage drive
  //1 engage driveTrain
  //2 engage both(sus)
  static int transmissionState;
  static int transmissionState_last;


  //0 = peg intaking
  //1 = arm down peg down
  //2 = arm down peg in neutral state
  //3 = high goal
  //4 = claw goal
  // good to go to next mode;
  static bool mechStateReached;

  

  
  
  public:
    static bool volt_enabled;
    static double flippyPid_speed;
    static double armPid_speed;

    static double turretPid_speed;

    constexpr const static double turretKp = 1.0;
    constexpr const static double turretKd = 0;

    constexpr const static double turrOffConst = 1.0;

    static double shotHeight;

    static double shotAngle;

    static double flySpeed;

    constexpr const static double changedist = 100;
    constexpr const static double flyConst = 1;

    constexpr const static double deltaheight = 25;

    constexpr const static double offsetConstant = 1;

    static bool highShot;

    static bool driveHold;
    static int mechMode;
    //0 = drive engaged
    //1 = neutral
    //2 = mech megaged
    //3 = both engaged at the same time (sus)
    static int shiftMode;

    static double getArmPos();
    static double getFlippyPos();
    
    // acts as a overall guidance for processes that take multiple steps to do
    static int mechTGT;
    //input for driver, lets them manually control the arm and stuff but only sometimes and hopefully with working interlocks(maybe)
    static double manualArmSpeed;

    static double manualPegSpeed;

    

    //the torque level for the arm, accepts 0-100;
    static double turretSpeed;
    static double armTorque;
    static bool driveBoost;
    static bool driverControl;
    static double leftDrive;
    static double rightDrive;
    static double armSpeed;
    static double lastArmSpeed;
    static double armHoldPosL;
    static double armHoldPosR;
    robotControl();
    static void controlPnem();
    static void driveTrain();
    static void controlShift();
    static void runRobotControl();
    static bool flippyPIDenabled;
    static bool armPIDenabled;
    //a value of 200 coasts
    static double flippyTGT;
    static double armTGT;
    
    static bool rezero_flippy;
    static double scale();

    static double flippyPID();
    static double armPID();
    static void mechControl();
    static void enableDrive();

    static void turretAim(double target[], double x_speed, double y_speed);


    //modifiers
};

#endif