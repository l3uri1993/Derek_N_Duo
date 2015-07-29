
#ifndef _nuovalib_h_
#define _nuovalib_h_

#include <inttypes.h>
#include "wiring_analog.h"
#include "sfr_defs.h"
// Motors pin defines
#define MOTOR1_PIN	11
#define MOTOR2_PIN	3
#define MOTOR3_PIN	6
#define MOTOR4_PIN	5

// Bit positions in the 74HCT595 shift register output
#define MOTOR1_A 2
#define MOTOR1_B 3
#define MOTOR2_A 1
#define MOTOR2_B 4
#define MOTOR4_A 0
#define MOTOR4_B 6
#define MOTOR3_A 5
#define MOTOR3_B 7

// Constants that the user passes in to the motor calls
#define FORWARD 1
#define BACKWARD 2
#define BRAKE 3
#define RELEASE 4

// Constants that the user passes in to the stepper calls
#define SINGLE 1
#define DOUBLE 2
#define INTERLEAVE 3
#define MICROSTEP 4


// Arduino pin names for interface to 74HCT595 latch
#define MOTORLATCH 12
#define MOTORCLK 4
#define MOTORENABLE 7
#define MOTORDATA 8


class AFMotorController
{
  public:
    AFMotorController(void);
    void enable(void);
    friend class AF_DCMotor;
    void latch_tx(void);
    uint8_t TimerInitalized;
};

class AF_DCMotor
{
 public:
  AF_DCMotor(uint8_t motornum);
  void run(uint8_t);
  void setSpeed(uint8_t);

 private:
  uint8_t motornum;
};

//uint8_t getlatchstate(void);


#endif