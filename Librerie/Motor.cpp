/**
* @file arduino_motor_driver.cpp
*/

#include "Motor.h"

Derek::Motor::Motor(int number)
	: MotorDriver(), motor(number), currentSpeed(0)
{
}

void Derek::Motor::setMotor(int speed)
{
	if (speed >= 0)
	{
		motor.setDirection(FORWARD);
		motor.setSpeed(speed);
		currentSpeed = speed;
		dir = 1;
	}
	else
	{
		motor.setDirection(BACKWARD);
		motor.setSpeed(-speed);
		currentSpeed = -speed;
		dir = 0;
	}
}


int Derek::Motor::getSpeed() const
{
	return currentSpeed;
}

int Derek::Motor::getDir() const
{
	return dir;
}

int Derek::Motor::getMotorNumber() const
{
	return motor.getMotorNum();
}