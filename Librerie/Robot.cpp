//---------------------------------------------------------
/**
//    @file   Robot.cpp
//    @brief  Implementation file for class Robot
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
  GoStraightPid(&PidStraightInput, &PidStraightOutput, &PidStraightSetpoint, STRAIGHT_KP, STRAIGHT_KI, STRAIGHT_KD, DIRECT),
  TurnPid(&PidTurnInput, &PidTurnOutput, &PidTurnSetpoint, TURN_KP, TURN_KI, TURN_KD, DIRECT),
  Scanner(StepperMotor, Button, DistanceSensor)
{
  ///SCANNER INITIALITAZION/////////////////////////////////////
  stateSelector = STARTING_POINT;
  pResults = &fullScanResults[0];
  for (int i = 0; i < SECTORS_FOR_FULL_SCAN; i++)
  {
    fullScanResults[i] = 0;
  }
  ///MOTOR POWER INITIALIZATION/////////////////////////////////
  RightMotorStraightPowerHigh = HIGH_MOTOR_SPEED;
  LeftMotorStraightPowerHigh = RightMotorStraightPowerHigh;
  RightMotorStraightPowerLow = LOW_MOTOR_SPEED;
  LeftMotorStraightPowerLow = RightMotorStraightPowerLow;
  ///PID CONTROL SETPOINT INITIALIZATION////////////////////////
  PidStraightSetpoint = 0;
  PidTurnSetpoint = 0;
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
    case QUICK_SCANNING_TO_CHOICE:
      Turn(90, 0, 0);                                                           ///Funzioni per provare rotazione e PIDStraight
      //Go_Straight(&LeftMotorStraightPowerHigh, &RightMotorStraightPowerHigh);
      Quick_Choice();
      break;

    case STARTING_POINT:
      Start();
      break;

    case FULL_SCANNING_TO_CHOICE:
      Full_Choice();
      break;

    default:
      break;
  }
}

///STARTING POINT
void Robot::Start()
{
  Scanner.start_up(BUTTON_DIRECTION_DISTANCE, START_UP_SPEED);
  Stop_Bot();
  stateSelector = QUICK_SCANNING_TO_CHOICE;
}

///FULL CHOICE
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
      Go_Straight(&LeftMotorStraightPowerLow, &RightMotorStraightPowerLow);
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

///QUICK CHOICE
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
    RightMotorStraightPowerHigh = HIGH_MOTOR_SPEED;
    LeftMotorStraightPowerHigh = HIGH_MOTOR_SPEED;
    Go_Straight(&LeftMotorStraightPowerLow, &RightMotorStraightPowerLow);
  }
  else
  {
    RightMotorStraightPowerLow = LOW_MOTOR_SPEED;
    LeftMotorStraightPowerLow = LOW_MOTOR_SPEED;
    Go_Straight(&LeftMotorStraightPowerHigh, &RightMotorStraightPowerHigh);
  }
}

void Robot::Go_Straight(int *LeftMotorSpeed, int *RightMotorSpeed)
{
  /*
  PidEvaluate(&GoStraightPid, &PidStraightInput, &PidStraightOutput);
  *LeftMotorSpeed = *LeftMotorSpeed + PidStraightOutput;
  *RightMotorSpeed = *RightMotorSpeed - PidStraightOutput;
  Serial.print("\n\t");
  Serial.print(*LeftMotorSpeed);
  Serial.print("\n\t");
  Serial.print(*RightMotorSpeed);
  */
  LeftMotor.setMotor(*LeftMotorSpeed);
  RightMotor.setMotor(*RightMotorSpeed);

}

void Robot::Stop_Bot()
{
  LeftMotor.setMotor(0);
  RightMotor.setMotor(0);
}

void Robot::Rotate_Left(int angle)
{
  Turn(angle, LEFT_MOTOR_LEFT_ROTATION_SPEED, RIGHT_MOTOR_LEFT_ROTATION_SPEED);
}

void Robot::Rotate_Right(int angle)
{
  Turn(angle, LEFT_MOTOR_RIGHT_ROTATION_SPEED, RIGHT_MOTOR_RIGHT_ROTATION_SPEED);
}

void Robot::Turn(int angle, int leftMotorPower, int rightMotorPower)
{
  digitalWrite(14, LOW);
  float startAngle = Gyroscope.CalculateAngle();
  float currentAngle = startAngle;
  float lastAngle = 0;

  LeftMotor.setMotor(leftMotorPower);
  RightMotor.setMotor(rightMotorPower);

  while (1)
  {
    while (abs(startAngle - currentAngle) < ((angle) * converter))///Giro fino a raggiungere angolo
    {
      lastAngle = currentAngle;
      currentAngle = Gyroscope.CalculateAngle();
    }
    if (abs(currentAngle - lastAngle) < (40 * converter) ) ///Rilevazione corretta, esco dal loop
    {
      break;
    }
    else
    {
      while (abs(currentAngle - lastAngle) > (40 * converter))///Scarto errore di rilevazione
      {
        currentAngle = Gyroscope.CalculateAngle();
      }
    }
  }
  Stop_Bot();
  digitalWrite(14, HIGH);
  Gyroscope.Reset();
}

void Robot::PidEvaluate(PID * myPid, float * input, float * output)
{
  for (int i = 0; i < 6; i++)             ///Faccio 6 letture consecutive per stabilizzare input (spreco 30 ms)
  {
    *input = Gyroscope.CalculateAngle();
  }
  Serial.print("\nPID INPUT\t");
  *input = *input / converter;
  Serial.print(*input);
  myPid->Compute();
  *output = (int) * output;
  Serial.print("\nPID OUTPUT\t");
  Serial.print(*output);
}

