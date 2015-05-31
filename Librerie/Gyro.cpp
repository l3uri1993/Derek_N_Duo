//---------------------------------------------------------
/**
//    @file    Gyro.cpp
//    @brief  Implementation file for class Gyro
*/
//---------------------------------------------------------

#include "Gyro.h"

////////////CONSTRUCTOR AND DESTRUCTOR//////////////////////////
/// Default Constructor
Gyro::Gyro()
{
  mpuIntStatus = 0;                  
  packetSize = 0;                
  fifoCount = 0;    
  for (int j = 0; j < 64; j++)
  {
    fifoBuffer[j] = 0;
  }
  for (int i = 0; i < 3; i++)
  {
    euler[i] = 0.00f;
  }
}

/// Default Destructor
Gyro::~Gyro()
{
}

/////////////GYRO CLASS METHODS IMPLEMENTATION//////////////////

void Gyro::GetAngle()
{
  while (fifoCount < packetSize)
  {    
    fifoCount = mpu.getFIFOCount();
  }

  mpuIntStatus = mpu.getIntStatus();
  fifoCount = mpu.getFIFOCount();

  if ((mpuIntStatus & 0x10) || fifoCount >= 1024)
  {
    mpu.resetFIFO();
  }
  else if (mpuIntStatus & 0x02)
  {
    while (fifoCount < packetSize)
    {
      fifoCount = mpu.getFIFOCount();
    }
    
    mpu.getFIFOBytes(fifoBuffer, packetSize);
    fifoCount -= packetSize;
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetEuler(euler, &q);
    Serial.print("\nangolo\t");
    Serial.print(euler[0] * 180 / M_PI);
  }
}

float Gyro::CalculateAngle()
{
  GetAngle();
  return euler[0];
}

void  Gyro::Reset()
{
  mpu.initialize();
  if (mpu.testConnection() == false)
  {
    Serial.println("\nRIAVVIA ALIMENTAZIONE\n\nSENSORE BLOCCATO PER MOTIVI IGNOTI\n");
    while (1);
  }

  // return status after each device operation (0 = success, !0 = error)
  uint8_t devStatus =  mpu.dmpInitialize();

  /// Gyro offset calculated
  mpu.setXGyroOffset(51);
  mpu.setYGyroOffset(-38);
  mpu.setZGyroOffset(22);
  mpu.setXAccelOffset(-1170);
  mpu.setYAccelOffset(2481);
  mpu.setZAccelOffset(1443);

  if (devStatus == 0)
  {
    mpu.setDMPEnabled(true);
    mpuIntStatus = mpu.getIntStatus();
    packetSize = mpu.dmpGetFIFOPacketSize();
    euler[0] = 0.00f;
    mpu.resetFIFO();
    Serial.println(F("MPU IS WORKING\n"));
  }
  else
  {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    Serial.print(F("DMP Initialization failed (code "));
    Serial.print(devStatus);
    Serial.println(F(")"));
    while (1);
  }
}
