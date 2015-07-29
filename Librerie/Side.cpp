/**
* @file arduino_motor_driver.cpp
*/

#include "Side.h"

Derek::Side::Side(uint8_t f,uint8_t r)	
	{
		FrontMotor= new AF_DCMotor(f);
		RearMotor= new AF_DCMotor(r);
		currentSpeed=0;
	}
	
void Derek::Side::setSide(int speed)
{
	if (speed > 0)
	{
		FrontMotor->setSpeed((uint8_t)speed);
		RearMotor->setSpeed((uint8_t)speed);
		FrontMotor->run(FORWARD);
		RearMotor->run(FORWARD);
		currentSpeed = speed;
		dir = 1;
	}
	
	else if (speed < 0)
	{
		FrontMotor->setSpeed((uint8_t)(-speed));
		RearMotor->setSpeed((uint8_t)(-speed));
		FrontMotor->run(BACKWARD);
		RearMotor->run(BACKWARD);
		currentSpeed = speed;
		dir = 0;
	}
		
	else
	{
		FrontMotor->run(RELEASE);
		RearMotor->run(RELEASE);
		currentSpeed = 0;
	}
}

int Derek::Side::getSpeed() const
{
	return currentSpeed;
}

uint8_t Derek::Side::getDir() const
{
	return dir;
}

