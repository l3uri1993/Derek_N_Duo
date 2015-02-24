/**
 * @file arduino_motor_driver.h
 * @brief motor driver for the Arduino Motor Shield.
 */

#ifndef _ARDUINO_MOTOR_DRIVER_DEF
#define _ARDUINO_MOTOR_DRIVER_DEF

#include "MotorDriver.h"
#include "AR_DCmotor.h"

namespace Derek
{
	class Motor : public MotorDriver
	{
	public:
		/**
		* @brief Class constructor.
		* @param number Motor number.
		*/
		Motor(int number);

		/**
		* @brief Set motor direction and motor speed according
		*   to speed is positive or negative.
		* @param speed Motor speed.
		*/
		virtual void setMotor(int speed);

		/**
		* @brief Return the current motor speed.
		* @return currentSpeed.
		*/
		virtual int getSpeed() const;

		/**
		* @brief Return the current motor direction.
		* @return dir.
		*/
		int getDir() const;

		/**
		* @brief Return the motor number.
		* @return motorNum.
		*/
		int getMotorNumber() const;

	private:
		AR_DCmotor motor;
		int currentSpeed;
		int dir;
	};
};

#endif