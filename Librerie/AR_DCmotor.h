/**
 * @file AR_DCmotor.h
 * @brief Arduino Motor shield library.
 */

#ifndef _AR_DCMOTOR_DEF
#define _AR_DCMOTOR_DEF

#if defined(ARDUINO) && ARDUINO >= 100
#include "Arduino.h"
#else
#include "WProgram.h"
#endif

#define DIRECTION_A 12											/**< motor A direction pin, HIGH --> forward, LOW --> backward */
#define DIRECTION_B 13											/**< motor B direction pin, HIGH --> forward, LOW --> backward */
#define BRAKE_A 9												/**< motor A brake pin, HIGH --> brake the DC motor */
#define BRAKE_B 8												/**< motor B brake pin, HIGH --> brake the DC motor */
#define PWM_A 3													/**< motor A speed pin, the shield control the speed by varying the PWM duty cycle values*/
#define PWM_B 11												/**< motor B speed pin, the shield control the speed by varying the PWM duty cycle values*/
#define CURRENT_SENSING_A 14									/**< motor A current sensing pin, you can measure the current going through the DC motor by reading this pin*/
#define CURRENT_SENSING_B 15									/**< motor B current sensing pin, you can measure the current going through the DC motor by reading this pin*/

#define BACKWARD 0
#define FORWARD 1
#define BRAKE 2

class AR_DCmotor
{
public:
	
	/**
	* @brief Class constructors.
	* @param motorNumber Motor number you want to initialize.
	* @param speed Motor speed, set by default to 0.
	*/
	~AR_DCmotor();
	AR_DCmotor(int motorNumber, int speed = 0);

	/**
	* @brief initialization of Direction, Brake, PWM pins.
	*/
	bool initialize();

	/**
	* @brief Control the motor number and set the motor speed.
	* @param speed Speed to set.
	*/
	void setSpeed(int speed);

	/**
	* @brief Control the motor number and set the motor direction.
	* @param dir Direction to set.
	*/
	void setDirection(int dir);

	/**
	* @brief Return the motor number.
	* @return motorNum.
	*/
	int getMotorNum() const;

	/**
	* @brief Return the current sensing.
	* @return Voltage proportional to the measured current: 1.65V per A.
	*  The max value can be returned is 3.3V. If return 100 there's an error.
	*/
	int getCurrentSensing() const;

private:
	int motorNum;
	int currSpeed;
};

#endif