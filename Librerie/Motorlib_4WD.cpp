
#include "nuovalib.h"
#include "Arduino.h"
#include "wiring_analog.h"
#include "sfr_defs.h"
#include <inttypes.h>


static uint8_t latch_state;
static int _writeResolution = 8;

static inline uint32_t mapResolution(uint32_t value, uint32_t from, uint32_t to) {
	if (from == to)
		return value;
	if (from > to)
		return value >> (from-to);
	else
		return value << (to-from);
}

AFMotorController::AFMotorController(void) {
    TimerInitalized = false;
}

void AFMotorController::enable(void) {
  // setup the latch
  /*
  LATCH_DDR |= _BV(LATCH);
  ENABLE_DDR |= _BV(ENABLE);
  CLK_DDR |= _BV(CLK);
  SER_DDR |= _BV(SER);
  */
  pinMode(MOTORLATCH, OUTPUT);
  pinMode(MOTORENABLE, OUTPUT);
  pinMode(MOTORDATA, OUTPUT);
  pinMode(MOTORCLK, OUTPUT);
  pinMode(MOTOR1_PIN,OUTPUT);
  pinMode(MOTOR2_PIN,OUTPUT);
  pinMode(MOTOR3_PIN,OUTPUT);
  pinMode(MOTOR4_PIN,OUTPUT);

  latch_state = 0;

  latch_tx();  // "reset"

  //ENABLE_PORT &= ~_BV(ENABLE); // enable the chip outputs!
  digitalWrite(MOTORENABLE, LOW);
}


void AFMotorController::latch_tx(void) {
  uint8_t i;

  //LATCH_PORT &= ~_BV(LATCH);
  digitalWrite(MOTORLATCH, LOW);

  //SER_PORT &= ~_BV(SER);
  digitalWrite(MOTORDATA, LOW);

  for (i=0; i<8; i++) {
    //CLK_PORT &= ~_BV(CLK);
    digitalWrite(MOTORCLK, LOW);

    if (latch_state & _BV(7-i)) {
      //SER_PORT |= _BV(SER);
      digitalWrite(MOTORDATA, HIGH);
    } else {
      //SER_PORT &= ~_BV(SER);
      digitalWrite(MOTORDATA, LOW);
    }
    //CLK_PORT |= _BV(CLK);
    digitalWrite(MOTORCLK, HIGH);
  }
  //LATCH_PORT |= _BV(LATCH);
  digitalWrite(MOTORLATCH, HIGH);
}

static AFMotorController MC;

/******************************************
               MOTORS
******************************************/


AF_DCMotor::AF_DCMotor(uint8_t num) {
  motornum = num;
 // uint32_t chan;
  MC.enable();
  
	// pmc_enable_periph_clk(PWM_INTERFACE_ID);
	// PWMC_ConfigureClocks(PWM_FREQUENCY * PWM_MAX_DUTY_CYCLE, 0, VARIANT_MCK);
			

  switch (num) {
  case 1:
	// Setup PWM for this pin
	// PIO_Configure(g_APinDescription[MOTOR1_PIN].pPort,
			// g_APinDescription[MOTOR1_PIN].ulPinType,
			// g_APinDescription[MOTOR1_PIN].ulPin,
			// g_APinDescription[MOTOR1_PIN].ulPinConfiguration);
	
	analogWrite(MOTOR1_PIN,0);
    // chan = g_APinDescription[MOTOR1_PIN].ulPWMChannel;
	// PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
	// PWMC_SetDutyCycle(PWM_INTERFACE, chan, 0);
	// PWMC_DisableChannel(PWM_INTERFACE, chan);
    latch_state &= ~_BV(MOTOR1_A) & ~_BV(MOTOR1_B); // set both motor pins to 0
    MC.latch_tx();
    break;
  case 2:
  	// Setup PWM for this pin
	// PIO_Configure(g_APinDescription[MOTOR2_PIN].pPort,
			// g_APinDescription[MOTOR2_PIN].ulPinType,
			// g_APinDescription[MOTOR2_PIN].ulPin,
			// g_APinDescription[MOTOR2_PIN].ulPinConfiguration);

	analogWrite(MOTOR2_PIN,0);
	// chan = g_APinDescription[MOTOR2_PIN].ulPWMChannel;
	// PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
	// PWMC_SetDutyCycle(PWM_INTERFACE, chan, 0);
	// PWMC_DisableChannel(PWM_INTERFACE, chan);
    latch_state &= ~_BV(MOTOR2_A) & ~_BV(MOTOR2_B); // set both motor pins to 0
    MC.latch_tx();
    break;
  case 3:
  	// PIO_Configure(g_APinDescription[MOTOR3_PIN].pPort,
			// g_APinDescription[MOTOR3_PIN].ulPinType,
			// g_APinDescription[MOTOR3_PIN].ulPin,
			// g_APinDescription[MOTOR3_PIN].ulPinConfiguration);
	
	analogWrite(MOTOR3_PIN,0);
	// chan = g_APinDescription[MOTOR3_PIN].ulPWMChannel;
	// PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
	// PWMC_SetDutyCycle(PWM_INTERFACE, chan, 0);
	// PWMC_DisableChannel(PWM_INTERFACE, chan);
    latch_state &= ~_BV(MOTOR3_A) & ~_BV(MOTOR3_B); // set both motor pins to 0
    MC.latch_tx();
    break;
  case 4:
  	// PIO_Configure(g_APinDescription[MOTOR4_PIN].pPort,
			// g_APinDescription[MOTOR4_PIN].ulPinType,
			// g_APinDescription[MOTOR4_PIN].ulPin,
			// g_APinDescription[MOTOR4_PIN].ulPinConfiguration);
	
	analogWrite(MOTOR4_PIN,0);
    // chan = g_APinDescription[MOTOR4_PIN].ulPWMChannel;
	// PWMC_ConfigureChannel(PWM_INTERFACE, chan, PWM_CMR_CPRE_CLKA, 0, 0);
	// PWMC_SetDutyCycle(PWM_INTERFACE, chan, 0);
	// PWMC_DisableChannel(PWM_INTERFACE, chan);
    latch_state &= ~_BV(MOTOR4_A) & ~_BV(MOTOR4_B); // set both motor pins to 0
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::run(uint8_t cmd) {
  uint8_t a, b;
  switch (motornum) {
  case 1:
    a = MOTOR1_A; b = MOTOR1_B; break;
  case 2:
    a = MOTOR2_A; b = MOTOR2_B; break;
  case 3:
    a = MOTOR3_A; b = MOTOR3_B; break;
  case 4:
    a = MOTOR4_A; b = MOTOR4_B; break;
  default:
    return;
  }
  
  switch (cmd) {
  case FORWARD:
    latch_state |= _BV(a);
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  case BACKWARD:
    latch_state &= ~_BV(a);
    latch_state |= _BV(b); 
    MC.latch_tx();
    break;
  case RELEASE:
    latch_state &= ~_BV(a);     // A and B both low
    latch_state &= ~_BV(b); 
    MC.latch_tx();
    break;
  }
}

void AF_DCMotor::setSpeed(uint8_t speed) {
	uint32_t ulValue;
	uint32_t chan;
	switch (motornum) {
		case 1:
		
		analogWrite(MOTOR1_PIN,speed);
		// chan = g_APinDescription[MOTOR1_PIN].ulPWMChannel;
		// Serial.print("chan 1: ");
		// Serial.println(chan);
		// ulValue = mapResolution(speed, _writeResolution, DACC_RESOLUTION);
		// Serial.print("ulValue 1: ");
		// Serial.println(ulValue);
		// PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue); 
		break;
		case 2:
		
		analogWrite(MOTOR2_PIN,speed);
		// chan = g_APinDescription[MOTOR2_PIN].ulPWMChannel;
		// Serial.print("chan 2: ");
		// Serial.println(chan);
		// ulValue = mapResolution(speed, _writeResolution, DACC_RESOLUTION);
		// PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue); 
		break;
		case 3:
		
		analogWrite(MOTOR3_PIN,speed);
		// chan = g_APinDescription[MOTOR3_PIN].ulPWMChannel;
		// Serial.print("chan 3: ");
		// Serial.println(chan);
		// ulValue = mapResolution(speed, _writeResolution, DACC_RESOLUTION);
		// PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue); 
		break;
		case 4:
		
		analogWrite(MOTOR4_PIN,speed);
		// chan = g_APinDescription[MOTOR4_PIN].ulPWMChannel;
		// Serial.print("chan 4: ");
		// Serial.println(chan);
		// ulValue = mapResolution(speed, _writeResolution, DACC_RESOLUTION);  
		// PWMC_SetDutyCycle(PWM_INTERFACE, chan, ulValue); 
		break;
		}
}
