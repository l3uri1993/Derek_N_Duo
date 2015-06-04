/**
	@file Button_driver.h
	@brief Header file for class Button_driver
*/

///Including Arduino library
#include <Arduino.h>

///Protection from redefinition error
#ifndef Button_driver_DEF
#define Button_driver_DEF

/**
	@class Button_driver

	@brief Button device definition for the Derek robot
*/

namespace Derek
{
	class Button_driver
	{

	private:

	protected:

	public:

		///Constructors
		Button_driver();
		Button_driver(Button_driver &x);

		///Destructor
		virtual ~Button_driver();

		///Operator = overloading
		int operator =(Button_driver &x);

		///Init functions
		int init();
		int init(Button_driver &x);

		///Reset functions
		int reset();

		///Pointers nullifier function
		int nullifier_pointers();
		
		///Acquisition button state function
		virtual boolean acquisition_button_state() = 0;

	};
};

#endif
