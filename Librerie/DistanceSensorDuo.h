
/**
	@file DistanceSensorDuo.h

	@brief Header file for class DistanceSensorDuo
*/

///Including Arduino library
#include <Arduino.h>

///Including father class
#include "DistanceSensorDriver.h"

///Protection from redefinition error
#ifndef DISTANCE_SENSOR_DUO_DEF
#define DISTANCE_SENSOR_DUO_DEF

/**
	@class DistanceSensorDuo

	@brief Ultrasonic sensor (HC-SR04) driver for compatibility with Arduino Due
*/

#define MAX_TIME_SAFE 600 ///Max echo time delay accepted (usually 450 us) ritardo massimo del segnale di echo
#define TRIGGER_DELAY_CLEANING 10 ///Time (in us) neccessary to clean the trigger signal before sending the trigger pulse (copying from NewPing library)
#define TRIGGER_PULSE_DELAY 15 ///Trigger pulse duration (in us)
#define TIME_NO_OBSTACLE 38000 ///Max echo signal duration (in us)
#define TIME_FOR_NEXT_TRIGGER 60000 ///Time necessary between two trigger pulses (in us)
#define TIME_ANOMALY 40000 ///Time selected to control the possible anomaly generated from the HC_SR04 (in us)

namespace Derek
{
	class DistanceSensorDuo :public DistanceSensorDriver
	{

	private:
		
		///Identifier of the trigger pin
		byte trigger_pin;
		///Identifier of the echo pin
		byte echo_pin;

	protected:

	public:

		///Constructors
		DistanceSensorDuo();
		DistanceSensorDuo(DistanceSensorDuo &x);

		///Manual constructor
		DistanceSensorDuo(byte t_pin, byte e_pin, unsigned int max_d);

		///Destructor
		virtual ~DistanceSensorDuo();

		///Operator = overloading
		int operator =(DistanceSensorDuo &x);

		///Init functions
		int init();
		int init(DistanceSensorDuo &x);
		int init(byte t_pin, byte e_pin, unsigned int max_d);

		///Reset functions
		int reset();

		///Pointers nullifier function
		int nullifier_pointers();

		///Function to get the distance that the ultrasonic sensor sees
		virtual unsigned int getDistance();

		///Function to resolve the anomaly that occurs with the ultrasonic sensor HC-SR04
		virtual unsigned int getDistance_fixed();
	};
};
#endif