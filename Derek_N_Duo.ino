#include <Wire.h>
#include "Scanner_ultrasonic.h"
#include "Motor.h"
#include "I2Cdev.h"
#include "helper_3dmath.h"
#include "MPU6050_6Axis_MotionApps20.h"

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
#define LEFT_HIGH_MOTOR_SPEED 210
#define RIGHT_HIGH_MOTOR_SPEED 215
#define LEFT_LOW_MOTOR_SPEED 150
#define RIGHT_LOW_MOTOR_SPEED 165

//Times to select the angle rotation
#define A90_DEGREES 78 ///Time to make a rotation of 90 degrees
#define LEFT 64///Time to make a rotation of 72 degrees
#define RIGHT 64
#define CENTRAL_LEFT 32///Time to make a rotation of 36 degrees
#define CENTRAL_RIGHT 32

//Speeds to control the left rotation
#define LEFT_MOTOR_LEFT_ROTATION_SPEED -80
#define RIGHT_MOTOR_LEFT_ROTATION_SPEED 80

//Speeds to control the right rotation
#define LEFT_MOTOR_RIGHT_ROTATION_SPEED 80
#define RIGHT_MOTOR_RIGHT_ROTATION_SPEED -80

/////////////////////////////////////////////////

MPU6050 mpu;                           // mpu interface object

int buffersize=1000;     //Amount of readings used to average, make it higher to get more precision but sketch will be slower  (default:1000)
int acel_deadzone=8;     //Acelerometer error allowed, make it lower to get more precision, but sketch may not converge  (default:8)
int giro_deadzone=1;     //Giro error allowed, make it lower to get more precision, but sketch may not converge  (default:1)

int16_t ax, ay, az,gx, gy, gz;
int mean_ax,mean_ay,mean_az,mean_gx,mean_gy,mean_gz;
int ax_offset,ay_offset,az_offset,gx_offset,gy_offset,gz_offset;

bool dmpReady = false;                 // set true if DMP init was successful
uint8_t mpuIntStatus;                  // mpu statusbyte
uint8_t devStatus;                     // device status    
uint16_t packetSize;                   // estimated packet size  
uint16_t fifoCount;                    // fifo buffer size   
uint8_t fifoBuffer[64];                // fifo buffer 

Quaternion q;                          // quaternion for mpu output
float euler[3] = {0.0f,0.0f,0.0f};     // yaw pitch roll values
float rota = 0.0f;
float start = 0.0f;
                                                                   //int i = 0;

volatile bool mpuInterrupt = false;    //interrupt flag

/////////////////////////////////////////////////

class Robot
{
private:

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
		:leftMotor(LEFT_MOTOR_INIT), rightMotor(RIGHT_MOTOR_INIT),
		distanceSensor(DISTANCE_SENSOR_INIT),
		stepper(STEPPER_STEPS, STEPPER_PIN1, STEPPER_PIN2, STEPPER_PIN3, STEPPER_PIN4),
		button(BUTTON_PIN, DEBOUNCE_DELAY),
		scanner(stepper, button, distanceSensor)
	{
		stateSelector = STARTING_POINT;

		pResults = &fullScanResults[0];

		for (int i = 0; i<SECTORS_FOR_FULL_SCAN; i++)
		{
			fullScanResults[i] = 0;
		}
	}

	int run()
{
  
  if (stateSelector == STARTING_POINT)
		{
			start();
			return 0;
		}

		if (stateSelector == FULL_SCANNING_TO_CHOICE)
		{
			full_choice();
			return 0;
		}

		if (stateSelector == QUICK_SCANNING_TO_CHOICE)
		{
			quick_choice();
			return 0;
		}

		return 0;
		
}

	//STARTING POINT
	int start()
	{
		scanner.start_up(BUTTON_DIRECTION_DISTANCE, START_UP_SPEED);

		stop_bot();

		stateSelector = QUICK_SCANNING_TO_CHOICE;

		return 0;
	}

	//FULL CHOICE
	int full_choice()
	{
		// Identifier to keep in memory the sector with the max value
		int maxControl = 0;
		unsigned int maxVal = 0;

		//scanner.start_up(BUTTON_DIRECTION_DISTANCE,START_UP_SPEED);

		scanner.full_scan(SECTORS_FOR_FULL_SCAN, ACQUISITIONS_PER_SECTOR, SCAN_LENGHT, FULL_SCAN_SPEED, pResults);

		//    scanner.start_up(BUTTON_DIRECTION_DISTANCE,START_UP_SPEED);

		// Check if all sectors are occupied by obstacles
		for (int i = 0; i<SECTORS_FOR_FULL_SCAN; i++)
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
			return 0;
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

		return 0;
	}

	int quick_choice()
	{
 
                checkturn(A90_DEGREES); //AGGIUNTA PER TEST
  
		int x = scanner.quick_scan(SCANS_NUMBER);

		if (x <= TOO_CLOSE)
		{
			stop_bot();

			stateSelector = FULL_SCANNING_TO_CHOICE;

			return 0;
		}
		else if (x <= CLOSE)
		{
			go_straight(LEFT_LOW_MOTOR_SPEED, RIGHT_LOW_MOTOR_SPEED);

			return 0;
		}

		go_straight(LEFT_HIGH_MOTOR_SPEED, RIGHT_HIGH_MOTOR_SPEED);

		return 0;
	}

	int go_straight(int leftMotorSpeed, int rightMotorSpeed)
	{
		leftMotor.setMotor(leftMotorSpeed);
		rightMotor.setMotor(rightMotorSpeed);

		return 0;
	}

	int stop_bot()
	{
		leftMotor.setMotor(0);
		rightMotor.setMotor(0);

		return 0;
	}

	int rotate_left(int angle)
	{
		leftMotor.setMotor(LEFT_MOTOR_LEFT_ROTATION_SPEED);
		rightMotor.setMotor(RIGHT_MOTOR_LEFT_ROTATION_SPEED);
                
                checkturn(angle);

		//delay(angle);

		stop_bot();

		return 0;
	}

	int rotate_right(int angle)
	{
		leftMotor.setMotor(LEFT_MOTOR_RIGHT_ROTATION_SPEED);
		rightMotor.setMotor(RIGHT_MOTOR_RIGHT_ROTATION_SPEED);

                checkturn(angle);
                
		//delay(angle);

		stop_bot();

		return 0;
	}

};

Robot mybot;

void setup()
{
    // join I2C bus (I2Cdev library doesn't do this automatically)
    #if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
        Wire.begin();
    #elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
        Fastwire::setup(400, true);
    #endif
    
    pinMode(10, OUTPUT);
    pinMode(9, OUTPUT);
    pinMode(8, OUTPUT);
    digitalWrite(10,LOW);
    digitalWrite(9,LOW);
    digitalWrite(8,LOW);

    Serial.begin(9600);
    Serial.println(F("Initializing I2C devices..."));
    mpu.initialize();
    Serial.println(F("Testing device connections..."));
    if (mpu.testConnection() == false)
    {
      while(1)
    {
    Serial.println(F("RIAVVIA ALIMENTAZIONE"));
    digitalWrite(9,HIGH);
    }
    }
    
    Serial.println(F("Initializing DMP..."));
    devStatus = mpu.dmpInitialize();

    mpu.setXAccelOffset(0);
    mpu.setYAccelOffset(0);
    mpu.setZAccelOffset(0);
    mpu.setXGyroOffset(0);
    mpu.setYGyroOffset(0);
    mpu.setZGyroOffset(0);
  
    Serial.println("\nCalibrazione");
    meansensors();
    delay(1000);
    calibration();
    delay(1000);

    if (devStatus == 0)
    {
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);

        Serial.println(F("Enabling interrupt detection (Arduino external interrupt 2)..."));
        attachInterrupt(2, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();

        Serial.println(F("DMP ready! Waiting for first interrupt..."));
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
      
    }
    else
    { 
        Serial.print("DMP Initialization failed");
    }
}

void loop()
{
   mybot.run();                     
}

/////////////////////////////////////////////////////////FUNZIONI DA INCLUDERE NELLA CLASSE DEL GIROSCOPIO

void dmpDataReady() {
    mpuInterrupt = true;
}

void getangle()
{
    while (!mpuInterrupt && fifoCount < packetSize)  {}
  
    mpuInterrupt = false;
    mpuIntStatus = mpu.getIntStatus();
    fifoCount = mpu.getFIFOCount();
    
    if((mpuIntStatus & 0x10) || fifoCount >= 1024)
    { 
      
      mpu.resetFIFO(); 
    
    }
    
    else if(mpuIntStatus & 0x02)
      {
    
        while (fifoCount < packetSize) fifoCount = mpu.getFIFOCount();
  
        mpu.getFIFOBytes(fifoBuffer, packetSize);
      
        fifoCount -= packetSize;
    
        mpu.dmpGetQuaternion(&q, fifoBuffer);
        mpu.dmpGetEuler(euler, &q);
        Serial.print("\nangolo\t");
        Serial.print(euler[0] * 180/M_PI);
    }
}

int checkturn(int angolo)
{
    digitalWrite(8,HIGH);
    Serial.print("\nGuardo rotazione\n");
    delay(500);
    getangle();
    start = euler[0];
    rota = euler[0];
    while (abs(start - rota) < ((angolo) * M_PI/180))
    {
      getangle();
      rota = euler [0];
    }
                                                           /*  start = euler[0];
                                                               for (i=0;i<2;i++)
                                                               {
                                                               getangle();
                                                               rota = euler[0];
                                                               if (abs(start - rota) < ((angolo) * M_PI/180))
                                                                 {
                                                                   i=0;
                                                                   Serial.print("Errore evitato");
                                                                 }
                                                               while (abs(start - rota) < ((angolo) * M_PI/180))
                                                                 {
                                                                   getangle();
                                                                   rota = euler [0];
                                                                 }
                                                               mpu.resetFIFO();
                                                               } 
                                                           */
    Serial.print("RUOTATO\n");
    digitalWrite(8,LOW);
    reset();
    return 0;
 }
 
 void reset()
 {
    Serial.println(F("\nRESETTING\n"));
    mpu.initialize();
    if (mpu.testConnection() == false)
    {
      while(1)
    {
    digitalWrite(9,HIGH);
    }
    }
    
    devStatus = mpu.dmpInitialize();

    // make sure it worked (returns 0 if so)
    if (devStatus == 0)
    {
        mpu.setDMPEnabled(true);
        attachInterrupt(2, dmpDataReady, RISING);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        packetSize = mpu.dmpGetFIFOPacketSize();
      
    }
    euler[0] = 0.0f;
    start = 0.0f;
    rota = 0.0f;
    mpu.resetFIFO();
    delay(2000);
}

    
void meansensors(){
  long i=0,buff_ax=0,buff_ay=0,buff_az=0,buff_gx=0,buff_gy=0,buff_gz=0;

  while (i<(buffersize+101)){
    // read raw accel/gyro measurements from device
    mpu.getMotion6(&ax, &ay, &az, &gx, &gy, &gz);
    
    if (i>100 && i<=(buffersize+100)){ //First 100 measures are discarded
      buff_ax=buff_ax+ax;
      buff_ay=buff_ay+ay;
      buff_az=buff_az+az;
      buff_gx=buff_gx+gx;
      buff_gy=buff_gy+gy;
      buff_gz=buff_gz+gz;
    }
    if (i==(buffersize+100)){
      mean_ax=buff_ax/buffersize;
      mean_ay=buff_ay/buffersize;
      mean_az=buff_az/buffersize;
      mean_gx=buff_gx/buffersize;
      mean_gy=buff_gy/buffersize;
      mean_gz=buff_gz/buffersize;
    }
    i++;
    delay(2); //Needed so we don't get repeated measures
  }
}

void calibration(){
  ax_offset=-mean_ax/8;
  ay_offset=-mean_ay/8;
  az_offset=(16384-mean_az)/8;

  gx_offset=-mean_gx/4;
  gy_offset=-mean_gy/4;
  gz_offset=-mean_gz/4;
  while (1){
    int ready=0;
    mpu.setXAccelOffset(ax_offset);
    mpu.setYAccelOffset(ay_offset);
    mpu.setZAccelOffset(az_offset);

    mpu.setXGyroOffset(gx_offset);
    mpu.setYGyroOffset(gy_offset);
    mpu.setZGyroOffset(gz_offset);

    meansensors();
    Serial.println("...");

    if (abs(mean_ax)<=acel_deadzone) ready++;
    else ax_offset=ax_offset-mean_ax/acel_deadzone;

    if (abs(mean_ay)<=acel_deadzone) ready++;
    else ay_offset=ay_offset-mean_ay/acel_deadzone;

    if (abs(16384-mean_az)<=acel_deadzone) ready++;
    else az_offset=az_offset+(16384-mean_az)/acel_deadzone;

    if (abs(mean_gx)<=giro_deadzone) ready++;
    else gx_offset=gx_offset-mean_gx/(giro_deadzone+1);

    if (abs(mean_gy)<=giro_deadzone) ready++;
    else gy_offset=gy_offset-mean_gy/(giro_deadzone+1);

    if (abs(mean_gz)<=giro_deadzone) ready++;
    else gz_offset=gz_offset-mean_gz/(giro_deadzone+1);

    if (ready==6) break;
  }
}
