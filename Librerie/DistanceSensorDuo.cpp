
/**
	@file DistanceSensorDuo.cpp

	@brief Implementation file for class DistanceSensorDuo
*/

///Including the header file associated
#include "DistanceSensorDuo.h"

///Implementations

///Default constructor
Derek::DistanceSensorDuo::DistanceSensorDuo()
{
	nullifier_pointers();
	init();
}

///Copy constructor
Derek::DistanceSensorDuo::DistanceSensorDuo(DistanceSensorDuo &x)
{
	nullifier_pointers();
	init(x);
}

///Manual constructor
Derek::DistanceSensorDuo::DistanceSensorDuo(byte t_pin, byte e_pin, unsigned int max_d)
{
	reset();

	init(t_pin, e_pin, max_d);
}

///Destructor
Derek::DistanceSensorDuo::~DistanceSensorDuo()
{
	reset();
}

///Operator = overloading
int Derek::DistanceSensorDuo::operator =(DistanceSensorDuo &x)
{
	// /*
	reset();
	// */

	init(x);

	return 1;
}

///Init default initializer
int Derek::DistanceSensorDuo::init()
{
	reset();

	return 1;
}

///Init copy initializer
int Derek::DistanceSensorDuo::init(DistanceSensorDuo &x)
{
	reset();
	
	trigger_pin = x.trigger_pin;
	echo_pin = x.echo_pin;
	maxDistance = x.maxDistance;

	///Preparing I/O
	pinMode(trigger_pin, OUTPUT);
	pinMode(echo_pin, INPUT);

	return 1;
}

///Init manual initializer
int Derek::DistanceSensorDuo::init(byte t_pin, byte e_pin, unsigned int max_d)
{
	reset();

	trigger_pin = t_pin;
	echo_pin = e_pin;
	maxDistance = max_d;

	///Preparing I/O
	pinMode(trigger_pin, OUTPUT);
	pinMode(echo_pin, INPUT);

	return 1;
}

///Reset functions
int Derek::DistanceSensorDuo::reset()
{
	trigger_pin = 0;
	echo_pin = 0;
	maxDistance = 0;

	return 1;
}

///Pointers nullifier function
int Derek::DistanceSensorDuo::nullifier_pointers()
{

	return 1;
}

///Acquisition distance in cm
unsigned int Derek::DistanceSensorDuo::getDistance()
{
	
	///Preparing local variables

	///Temporary var to have a clean and understandable code
	unsigned long time = 0;
	///Counter to control the searching of echo signal rise
	unsigned long time_safe = 0;
	///Counter to control the searching of echo signal fall
	unsigned long time_controller = 0;
	///Counter to control the time between two trigger pulse
	unsigned long time_next_trigger = 0;
	///Counter to control the anomaly of HC_SR04
	unsigned long time_anomaly = 0;
	
	///Value that the function will return (if all OK)
	unsigned long distance = 0;

	///Trigger pulse

	///Cleaning the trigger signal
	digitalWrite(trigger_pin,LOW);
	delayMicroseconds(TRIGGER_DELAY_CLEANING);///Min 4 us
	///Sending the trigger pulse to comunicate with the ultrasonic sensor
	digitalWrite(trigger_pin,HIGH);
	delayMicroseconds(TRIGGER_PULSE_DELAY);///Min 10 us
	///Recleaning the trigger signal
	digitalWrite(trigger_pin, LOW);

	///Saving the time when the trigger signal is sent
	time = micros();
	///Initializing time safe
	time_safe = time + MAX_TIME_SAFE;
	///Initializing time for next trigger pulse
	time_next_trigger = time + TIME_FOR_NEXT_TRIGGER;
	
	///Searching the echo signal rise...
	while (digitalRead(echo_pin) != HIGH)
	{
		///...and checking to avoid an infinite loop
		if (micros() >= time_safe)
		{
			return 1;
		}
	}

	///Saving the instant
	time_controller = micros() + TIME_ANOMALY;

	///Searching the echo signal fall...
	while (digitalRead(echo_pin) != LOW)
	{
		if (micros() >= time_controller)
		{
			///...and checking that the period, in wich the echo signal is HIGH, is not longer than TIME_ANOMALY
			return 0;
		}
	}

	///Saving the time between the starting and the ending of HIGH state of echo signal 
	time_controller = micros() + TIME_ANOMALY - time_controller;

	///If the "HIGH echo" period is longer than the max time possible, it warns the user that there is an ERROR returning 0...
	if (time_controller > TIME_NO_OBSTACLE)
	{
		return maxDistance;
	}
	else
	{
		///...else it converts from microseconds to centimeters
		distance = (time_controller * 17);
		distance = distance + 500;
		distance = distance / 1000;
	}

	///Cleaning the result
	if (distance > maxDistance)
	{
		distance = maxDistance;
	}

	///Waiting the end of time necessary for a correct use of the next trigger
	while (micros() < time_next_trigger)
	{
		///NOOOOOOOOOP.......
	}

	return (unsigned int)distance;
}

///Fixing the anomaly of the ultrasonic distance sensor
unsigned int Derek::DistanceSensorDuo::getDistance_fixed()
{
	unsigned int x;
	unsigned int y;

	x = getDistance();

	if (x == 0)
	{
		x = maxDistance;
	}

	if (x >= 3 && x <= 5)
	{
		y = getDistance();

		if (y == 0)
		{
			return maxDistance;
		}
		
		x = (x + y) / 2;
	}

	return x;
}