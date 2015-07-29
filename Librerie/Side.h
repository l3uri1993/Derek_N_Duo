
#ifndef _ARDUINO_MOTOR_DRIVER_DEF
#define _ARDUINO_MOTOR_DRIVER_DEF

#include "Arduino.h"
#include "nuovalib.h"

namespace Derek
{
	class Side //: public MotorDriver
	{public:
		/**
		* @brief Class constructor.
		* @param number Motor number.
		*/
		Side(uint8_t f,uint8_t r);

		/**
		* @brief Set motor direction and motor speed according
		*   to speed is positive or negative.
		* @param speed Motor speed.
		*/
		virtual void setSide(int speed);

		/**
		* @brief Return the current motor speed.
		* @return currentSpeed.
		*/
		virtual int getSpeed() const;

		/**
		* @brief Return the current motor direction.
		* @return dir.
		*/
		uint8_t getDir() const;

		

	private:
		AF_DCMotor* FrontMotor;
		AF_DCMotor* RearMotor;	
		int currentSpeed;
		uint8_t dir;
	};
};

#endif
