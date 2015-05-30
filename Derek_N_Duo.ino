#include <Wire.h>
#include "Scanner_ultrasonic.h"
#include "Motor.h"
#include "Gyro.h"

using namespace Derek;

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

//States
#define STARTING_POINT 0
#define FULL_SCANNING_TO_CHOICE 1
#define QUICK_SCANNING_TO_CHOICE 2

//Start up values
#define BUTTON_DIRECTION_DISTANCE 63
#define START_UP_SPEED 20

//Full scan values
#define SECTORS_FOR_FULL_SCAN 5
#define ACQUISITIONS_PER_SECTOR 3
#define SCAN_LENGHT 100
#define FULL_SCAN_SPEED 30
#define CENTRAL_CECK_DELAY 200

//Quick scan values
#define SCANS_NUMBER 1

//Distance to consider the presence of an obstacle
#define TOO_CLOSE 40
#define CLOSE 70

//Motors Speeds for the quick scan
#define LEFT_HIGH_MOTOR_SPEED 175
#define RIGHT_HIGH_MOTOR_SPEED 180
#define LEFT_LOW_MOTOR_SPEED 150
#define RIGHT_LOW_MOTOR_SPEED 165

//Times to select the angle rotation
#define A90_DEGREES 88 ///Time to make a rotation of 90 degrees
#define LEFT 70///Time to make a rotation of 72 degrees
#define RIGHT 70
#define CENTRAL_LEFT 35///Time to make a rotation of 36 degrees
#define CENTRAL_RIGHT 35

//Speeds to control the left rotation
#define LEFT_MOTOR_LEFT_ROTATION_SPEED -130
#define RIGHT_MOTOR_LEFT_ROTATION_SPEED 130

//Speeds to control the right rotation
#define LEFT_MOTOR_RIGHT_ROTATION_SPEED 130
#define RIGHT_MOTOR_RIGHT_ROTATION_SPEED -130

/////////////////////////////////////////////////

class Robot
{
  private:

    Gyro Gyroscope;
    Motor leftMotor;
    Motor rightMotor;
    Stepper stepper;
    Button_debounced button;
    DistanceSensorDuo distanceSensor;
    Scanner_ultrasonic scanner;
    
    int stateSelector;
    unsigned int fullScanResults[SECTORS_FOR_FULL_SCAN];
    unsigned int *pResults;

  public:

    //Constructor
    Robot()
      : leftMotor(LEFT_MOTOR_INIT), rightMotor(RIGHT_MOTOR_INIT),        
        distanceSensor(DISTANCE_SENSOR_INIT),
        stepper(STEPPER_STEPS, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4),
        button(BUTTON_PIN, DEBOUNCE_DELAY),
        Gyroscope(),
        scanner(stepper, button, distanceSensor)
    {
      stateSelector = STARTING_POINT;
      pResults = &fullScanResults[0];
      for (int i = 0; i < SECTORS_FOR_FULL_SCAN; i++)
      {
        fullScanResults[i] = 0;
      }
    }

    void setup()
    {
      Serial.begin(9600);
      Wire.begin();
      Gyroscope.Reset();
    }

    void run()
    {
      Gyroscope.Checkturn(35); //AGGIUNTA PER TEST
      switch (stateSelector)
      {
        case STARTING_POINT:
          start();
          break;

        case FULL_SCANNING_TO_CHOICE:
          full_choice();
          break;

        case QUICK_SCANNING_TO_CHOICE:
          quick_choice();
          break;

        default:
          break;
      }
    }

    //STARTING POINT
    void start()
    {
      scanner.start_up(BUTTON_DIRECTION_DISTANCE, START_UP_SPEED);
      stop_bot();
      stateSelector = QUICK_SCANNING_TO_CHOICE;
    }

    //FULL CHOICE
    void full_choice()
    {
      // Identifier to keep in memory the sector with the max value
      int maxControl = 0;
      unsigned int maxVal = 0;   

      scanner.full_scan(SECTORS_FOR_FULL_SCAN, ACQUISITIONS_PER_SECTOR, SCAN_LENGHT, FULL_SCAN_SPEED, pResults);

      // Check if all sectors are occupied by obstacles
      for (int i = 0; i < SECTORS_FOR_FULL_SCAN; i++)
      {
        if (fullScanResults[i] > maxVal)
        {
          maxVal = fullScanResults[i];
          maxControl = i;
        }
      }

      // If the max value is lower than the value used to consider an obstacle as close, the robot rotate 90 degrees
      if (maxVal < TOO_CLOSE)
      {
        rotate_right(A90_DEGREES);
      }

      switch (maxControl)
      {
        case 0://Left
          rotate_left(LEFT);
          break;

        case 1://Central left
          rotate_left(CENTRAL_LEFT);
          break;

        case 2://Center
          go_straight(LEFT_LOW_MOTOR_SPEED, RIGHT_LOW_MOTOR_SPEED);
          delay(CENTRAL_CECK_DELAY);
          stop_bot();
          break;

        case 3://Central right
          rotate_right(CENTRAL_RIGHT);
          break;

        case 4://Right
          rotate_right(RIGHT);
          break;

        default:
          break;
      }
      stateSelector = QUICK_SCANNING_TO_CHOICE;
    }

    void quick_choice()
    {
      int x = scanner.quick_scan(SCANS_NUMBER);

      if (x <= TOO_CLOSE)
      {
        stop_bot();

        stateSelector = FULL_SCANNING_TO_CHOICE;

      }
      else if (x <= CLOSE)
      {
        go_straight(LEFT_LOW_MOTOR_SPEED, RIGHT_LOW_MOTOR_SPEED);

      }
      
      go_straight(LEFT_HIGH_MOTOR_SPEED, RIGHT_HIGH_MOTOR_SPEED);
    }

    void go_straight(int leftMotorSpeed, int rightMotorSpeed)
    {
      leftMotor.setMotor(leftMotorSpeed);
      rightMotor.setMotor(rightMotorSpeed);
    }

    void stop_bot()
    {
      leftMotor.setMotor(0);
      rightMotor.setMotor(0);
    }

    void rotate_left(int angle)
    {
      leftMotor.setMotor(LEFT_MOTOR_LEFT_ROTATION_SPEED);
      rightMotor.setMotor(RIGHT_MOTOR_LEFT_ROTATION_SPEED);
      Gyroscope.Checkturn(angle);
      stop_bot();
    }

    void rotate_right(int angle)
    {
      leftMotor.setMotor(LEFT_MOTOR_RIGHT_ROTATION_SPEED);
      rightMotor.setMotor(RIGHT_MOTOR_RIGHT_ROTATION_SPEED);
      Gyroscope.Checkturn(angle);
      stop_bot();
    }

};

Robot mybot;

void setup()
{
  mybot.setup();
}

void loop()
{
  mybot.run();
}
