//---------------------------------------------------------
/**
//    @file   Robot.h
//    @brief  Header file for CLASS Robot
*/
//---------------------------------------------------------

#ifndef Robot__DEF
#define Robot__DEF

#include "Scanner_ultrasonic.h"
#include "Motor.h"
#include "Gyro.h"
#include "PID_v1.h"

using namespace Derek;

/**
@class Robot
@brief Robot class is Derek main class which coordinates all the others components
*/

///Value for the constructor
#define LEFT_MOTOR_INIT 2
#define RIGHT_MOTOR_INIT 1
#define DISTANCE_SENSOR_INIT 23,22,500
#define STEPPER_STEPS 200
#define STEPPER_PIN1 24
#define STEPPER_PIN2 28
#define STEPPER_PIN3 26
#define STEPPER_PIN4 30
#define BUTTON_PIN 32
#define DEBOUNCE_DELAY 2

///States
#define STARTING_POINT 0
#define FULL_SCANNING_TO_CHOICE 1
#define QUICK_SCANNING_TO_CHOICE 2

///Start up values
#define BUTTON_DIRECTION_DISTANCE 63
#define START_UP_SPEED 20

///Full scan values
#define SECTORS_FOR_FULL_SCAN 5
#define ACQUISITIONS_PER_SECTOR 3
#define SCAN_LENGHT 100
#define FULL_SCAN_SPEED 30
#define CENTRAL_CECK_DELAY 200

///Quick scan values
#define SCANS_NUMBER 1

///Distance to consider the presence of an obstacle
#define TOO_CLOSE 40
#define CLOSE 80

///Motors Speeds for the quick scan
#define HIGH_MOTOR_SPEED 200
#define LOW_MOTOR_SPEED 140

///Angles to rotate
#define A90_DEGREES 85 
#define LEFT 65
#define RIGHT 65
#define CENTRAL_LEFT 30
#define CENTRAL_RIGHT 30

///Speeds to control the left rotation
#define LEFT_MOTOR_LEFT_ROTATION_SPEED -70
#define RIGHT_MOTOR_LEFT_ROTATION_SPEED 70

///Speeds to control the right rotation
#define LEFT_MOTOR_RIGHT_ROTATION_SPEED 70
#define RIGHT_MOTOR_RIGHT_ROTATION_SPEED -70

//PID
#define STRAIGHT_KP 1
#define STRAIGHT_KI 5
#define STRAIGHT_KD 0

#define TURN_KP 1
#define TURN_KI 5
#define TURN_KD 0
/////////////////////////////////////////////////

class Robot
{
    ///////////////////////////// PUBLIC    Segment
  public:

    /// @name CONSTRUCTORS/DESTRUCTOR
    /// @{

    Robot();
    virtual ~Robot();

    //@}

    /// @name PUBLIC CLASS METHODS
    /// @{
    
    void setup();
    
    void run();
    
    //@}

    ///////////////////////////// PRIVATE    Segment
  private:

    Gyro Gyroscope;
    Motor LeftMotor;
    Motor RightMotor;
    Stepper StepperMotor;
    Button_debounced Button;
    DistanceSensorDuo DistanceSensor;
    Scanner_ultrasonic Scanner;
    PID GoStraightPid, TurnPid; 

    int stateSelector;
    unsigned int fullScanResults[SECTORS_FOR_FULL_SCAN];
    unsigned int *pResults;
    
    float PidStraightInput, PidStraightOutput, PidStraightSetpoint;
    float PidTurnInput, PidTurnOutput, PidTurnSetpoint;
    
    int LeftMotorStraightPowerHigh;
    int RightMotorStraightPowerHigh;
    int LeftMotorStraightPowerLow;
    int RightMotorStraightPowerLow;

    /// Constant for euler conversion calculated on creation of the object
    const float converter = M_PI / 180;

    /// @name PRIVATE CLASS METHODS
    /// @{

    void Start();

    void Full_Choice();

    void Quick_Choice();

    void Stop_Bot();

    void Go_Straight(int *leftMotorSpeed, int *rightMotorSpeed);

    void Rotate_Left(int angle);

    void Rotate_Right(int angle);

    void Turn(int angle, int leftMotorPower, int rightMotorPower);

    void PidEvaluate(PID * myPid, float *input, float *output);

    //@}
};

#endif
