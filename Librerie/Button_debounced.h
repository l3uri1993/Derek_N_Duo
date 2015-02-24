
/**
	@file Button_debounced.h

	@brief Header file for class Button_debounced
*/

///Including Arduino library
#include <Arduino.h>

///Including the parent class Button_driver
#include "Button_driver.h"

///Constant to define the number of cycles used in the acquisition_button_state function
#define CHECK_CYCLES 10
///Constant to define the number of cycles necessary to decide the result of the acquisition_button_state function 
#define DECISION_CYCLES 6

///Protection from redefinition error
#ifndef Button_debounced_DEF
#define Button_debounced_DEF

/**
	@class Button_debounced

	@brief Button driver for debounce algorithm implementation for Arduino
*/

namespace Derek
{

	class Button_debounced :public Button_driver
	{

	private:
		///Value to identify the pin associated to the button
		byte button_pin;
		
		///Variables to manage debounce algorithm
		
		///Last state of the button
		boolean last_button_state;
		///Response of the debounce function
		boolean debounce_response;
		///Last instant saved
		unsigned long last_debounce_instant;
		///Delay necessary to debounce
		unsigned long debounce_delay;

	protected:

	public:

		///Constructors
		Button_debounced();
		Button_debounced(Button_debounced &x);
		Button_debounced(byte b_pin,unsigned long d_delay);

		///Destructor
		virtual ~Button_debounced();

		///Operator = overloading
		int operator =(Button_debounced &x);

		///Init functions
		int init();
		int init(Button_debounced &x);
		int init(byte b_pin,unsigned long d_delay);

		///Reset functions
		int reset();
		
		///Pointers nullifier function
		int nullifier_pointers();
		
		///Acquisition button state function
		virtual boolean acquisition_button_state();
		
		///Debounce function
		virtual boolean debounce();
	};
};

#endif