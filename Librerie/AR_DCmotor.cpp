/**
* @file AR_DCmotor.cpp
*/
#include "AR_DCmotor.h"

AR_DCmotor::AR_DCmotor(int motorNumber, int speed)
{
	motorNum = motorNumber;
	currSpeed = speed;
	initialize();
}

AR_DCmotor::~AR_DCmotor()
{
}

bool AR_DCmotor::initialize()
{
	if (motorNum == 1)
	{
		pinMode(DIRECTION_A, OUTPUT);
		pinMode(BRAKE_A, OUTPUT);
		analogWrite(PWM_A, currSpeed);
		return 0;
	}

	else if (motorNum == 2)
	{
		pinMode(DIRECTION_B, OUTPUT);
		pinMode(BRAKE_B, OUTPUT);
		analogWrite(PWM_B, currSpeed);
		return 0;
	}

	else
	{
		return 1;
	}
}

void AR_DCmotor::setDirection(int dir)
{
	int a;
	if (motorNum == 1)
	{
		a = DIRECTION_A;
	}
	else if (motorNum == 2)
	{
		a = DIRECTION_B;
	}
	else
	{
		return;
	}
	switch (dir)
	{
	case BACKWARD:
		digitalWrite(a, LOW);
		break;
	case FORWARD:
		digitalWrite(a, HIGH);
		break;
	case BRAKE:
		setSpeed(0);
		break;
	default:
		return;
	}
	
}

void AR_DCmotor::setSpeed(int speed)
{
	if (motorNum == 1)
	{
		analogWrite(PWM_A, speed);
	}
	else if (motorNum == 2)
	{
		analogWrite(PWM_B, speed);
	}
	else
	{
		return;
	}
}

int AR_DCmotor::getMotorNum() const
{
	return motorNum;
}

int AR_DCmotor::getCurrentSensing() const
{
	if (motorNum == 1)
	{
		return analogRead(CURRENT_SENSING_A);
	}
	else if (motorNum == 2)
	{
		return analogRead(CURRENT_SENSING_B);
	}
	else
	{
		return 100;
	}
}