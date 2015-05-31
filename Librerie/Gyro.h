//---------------------------------------------------------
/**
//    @file   Gyro.h
//    @brief  Header file for CLASS Gyro
*/
//---------------------------------------------------------
#include "MPU6050_6Axis_MotionApps20.h"

#ifndef Gyro__DEF
#define Gyro__DEF

/**
@class Gyro
@brief Gyro class implements some funcionality of MPU6050 sensor
*/

class Gyro
{
    ///////////////////////////// PRIVATE   Segment
  private:

    MPU6050 mpu;                           // mpu interface object
    uint8_t mpuIntStatus;                  // mpu statusbyte
    uint16_t packetSize;                   // estimated packet size
    uint16_t fifoCount;                    // fifo buffer size
    uint8_t fifoBuffer[64];                // fifo buffer

    Quaternion q;                          // quaternion for mpu output
    float euler[3];                        // euler angle values

    ///Gets the real-time angle and stores it in euler[0]
    //NB: euler[0] is private member, not accessible from the outside of class.
    //Use "CalculateAngle" to get euler[0]
    void GetAngle();


    ///////////////////////////// PUBLIC    Segment
  public:

    /// @name CONSTRUCTORS/DESTRUCTOR
    /// @{

    /// Default constructor
    Gyro();
    virtual ~Gyro();

    //@}

    /// @name CLASS METHODS
    /// @{

    ///Return the value of realtime calculated angle from euler[0]
    float CalculateAngle();

    ///Reset MPU6050 to avoid drift after rotation
    void Reset();

    //@}
};

#endif
