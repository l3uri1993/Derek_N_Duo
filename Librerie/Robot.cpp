//---------------------------------------------------------
/**
//    @file    Gyro.cpp
//    @brief  Implementation file for class Gyro
*/
//---------------------------------------------------------

#include "Robot.h"

////////////CONSTRUCTOR AND DESTRUCTOR//////////////////////////

/// Default Constructor
Robot::Robot() : LeftMotor(LEFT_MOTOR_INIT), RightMotor(RIGHT_MOTOR_INIT),
  DistanceSensor(DISTANCE_SENSOR_INIT),
  StepperMotor(STEPPER_STEPS, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4),
  Button(BUTTON_PIN, DEBOUNCE_DELAY),
  Gyroscope(),
  Scanner(StepperMotor, Button, DistanceSensor)
{
  stateSelector = STARTING_POINT;
  pResults = &fullScanResults[0];
  for (int i = 0; i < SECTORS_FOR_FULL_SCAN; i++)
  {
    fullScanResults[i] = 0;
  }
}

/// Default Destructor
Robot::~Robot()
{
}

/////////////ROBOT CLASS METHODS IMPLEMENTATION//////////////////

void Robot::setup()
{
  Gyroscope.Reset();
}

void Robot::run()
{
  switch (stateSelector)
  {
    case STARTING_POINT:
      Serial.print("\nSTARTING_POINT\n");
      Start();
      break;

    case FULL_SCANNING_TO_CHOICE:
      Serial.print("\nFULL_SCANNING_TO_CHOICE\n");
      Full_Choice();
      break;

    case QUICK_SCANNING_TO_CHOICE:
      Serial.print("\nQUICK_SCANNING_TO_CHOICE\n");
      Quick_Choice();
      break;

    default:
      break;
  }
}

//STARTING POINT
void Robot::Start()
{
  Scanner.start_up(BUTTON_DIRECTION_DISTANCE, START_UP_SPEED);
  Stop_Bot();
  stateSelector = QUICK_SCANNING_TO_CHOICE;
}

//FULL CHOICE
void Robot::Full_Choice()
{
  // Identifier to keep in memory the sector with the max value
  int maxControl = 0;
  unsigned int maxVal = 0;

  Scanner.full_scan(SECTORS_FOR_FULL_SCAN, ACQUISITIONS_PER_SECTOR, SCAN_LENGHT, FULL_SCAN_SPEED, pResults);

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
    Rotate_Right(A90_DEGREES);
  }

  switch (maxControl)
  {
    case 0://Left
      Rotate_Left(LEFT);
      break;

    case 1://Central left
      Rotate_Left(CENTRAL_LEFT);
      break;

    case 2://Center
      Go_Straight(LEFT_LOW_MOTOR_SPEED, RIGHT_LOW_MOTOR_SPEED);
      delay(CENTRAL_CECK_DELAY);
      Stop_Bot();
      break;

    case 3://Central right
      Rotate_Left(CENTRAL_RIGHT);
      break;

    case 4://Right
      Rotate_Left(RIGHT);
      break;

    default:
      break;
  }
  stateSelector = QUICK_SCANNING_TO_CHOICE;
}

void Robot::Quick_Choice()
{
  int x = Scanner.quick_scan(SCANS_NUMBER);

  if (x <= TOO_CLOSE)
  {
    Stop_Bot();

    stateSelector = FULL_SCANNING_TO_CHOICE;

  }
  else if (x <= CLOSE)
  {
    Go_Straight(LEFT_LOW_MOTOR_SPEED, RIGHT_LOW_MOTOR_SPEED);

  }

  Go_Straight(LEFT_HIGH_MOTOR_SPEED, RIGHT_HIGH_MOTOR_SPEED);
}

void Robot::Go_Straight(int LeftMotorSpeed, int RightMotorSpeed)
{
  LeftMotor.setMotor(LeftMotorSpeed);
  RightMotor.setMotor(RightMotorSpeed);
}

void Robot::Stop_Bot()
{
  LeftMotor.setMotor(0);
  RightMotor.setMotor(0);
}

void Robot::Rotate_Left(int angle)
{
  LeftMotor.setMotor(LEFT_MOTOR_LEFT_ROTATION_SPEED);
  RightMotor.setMotor(RIGHT_MOTOR_LEFT_ROTATION_SPEED);

  Turn(angle);
}

void Robot::Rotate_Right(int angle)
{
  LeftMotor.setMotor(LEFT_MOTOR_RIGHT_ROTATION_SPEED);
  RightMotor.setMotor(RIGHT_MOTOR_RIGHT_ROTATION_SPEED);

  Turn(angle);
}

void Robot::Turn(int angle)
{
  Serial.print("\nIn rotazione...");
  float startAngle = Gyroscope.CalculateAngle();
  float currentAngle = startAngle;
  while (abs(startAngle - currentAngle) < ((angle) * M_PI / 180))
  {
    currentAngle = Gyroscope.CalculateAngle();
  }
  Stop_Bot();
  Serial.print("\nRUOTATO\n");
  Gyroscope.Reset();
}
