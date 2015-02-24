
/**
	@file Scanner_ultrasonic.h

	@brief Header file for class Scanner_ultrasonic
*/

///Including Arduino library
#include <Arduino.h>

///Including parent class Scanner_driver
#include "Scanner_driver.h"

///Including classes for dynamic composition
#include "Stepper.h"
#include "Button_debounced.h"
#include "DistanceSensorDuo.h"

///Defining constants

///Constant to define the waiting time (in ms) to use correctly the ultrasonic sensor
#define WAITING_TIME_SENSOR 100
///Angle for quick scan
#define QUICK_SCAN_DISTANCE 10
///Speed for quick scan
#define QUICK_SCAN_SPEED 50
///Speed for full scan initialization
#define FULL_SCAN_SPEED_INITIALIZING 100

///Protection from redefinition error
#ifndef Scanner_ultrasonic_DEF
#define Scanner_ultrasonic_DEF

/**
	@class Scanner_ultrasonic

	@brief Scanner driver to pilot a "scanner" composed by a stepper, a sensor (HC-SR04) and a limit switch (button)
*/

namespace Derek
{

	class Scanner_ultrasonic :public Scanner_driver
	{

	private:
		///Variable to memorize direction for quick scan
		byte quick_scan_position;
		
		///Pointer to control a Stepper object
		Stepper *scanner_stepper;
		///Pointer to control a Button_debounced object
		Button_debounced *scanner_button;
		///Pointer to control a DistanceSensor object
		DistanceSensorDuo *scanner_sensor;

	protected:

	public:

		///Constructors
		Scanner_ultrasonic();
		Scanner_ultrasonic(Scanner_ultrasonic &x);
		Scanner_ultrasonic(Stepper &sc_stepper,Button_debounced &sc_button,DistanceSensorDuo &sc_sensor);

		///Destructor
		virtual ~Scanner_ultrasonic();

		///Operator = overloading
		int operator =(Scanner_ultrasonic &x);

		///Init functions
		int init();
		int init(Scanner_ultrasonic &x);
		int init(Stepper &sc_stepper,Button_debounced &sc_button,DistanceSensorDuo &sc_sensor);

		///Reset functions
		int reset();
		
		///Pointers nullifier function
		int nullifier_pointers();
		
		///Start up function
		virtual int start_up(int button_direction_distance,unsigned int stepper_speed);
		
		///Rotation for quick scan function
		virtual int rotation(unsigned int quick_scan_distance, unsigned int quick_scan_speed);
		
		///Quick scan function
		virtual int quick_scan(unsigned int number_scans);

		///Full scan function
		virtual unsigned int full_scan(byte sectors, unsigned int points_per_sector, unsigned int scan_lenght, unsigned int st_speed, unsigned int *p_results);
	};
};

#endif
