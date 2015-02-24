
/**
	@file Button_debounced.cpp

	@brief Implementation file for class Button_debounced
*/

///Including the header file associated
#include "Button_debounced.h"

///Implementations

///Default constructor
Derek::Button_debounced::Button_debounced()
{
	nullifier_pointers();
	init();
}

///Copy constructor
Derek::Button_debounced::Button_debounced(Button_debounced &x)
{
	nullifier_pointers();
	init(x);
}

///Manual constructor
Derek::Button_debounced::Button_debounced(byte b_pin,unsigned long d_delay)
{
	nullifier_pointers();
	init(b_pin,d_delay);
}

///Destructor
Derek::Button_debounced::~Button_debounced()
{
	reset();
}

///Operator = overloading
int Derek::Button_debounced::operator =(Button_debounced &x)
{
	// /*
	reset();
	// */

	init(x);

	return 1;
}

///Init default initializer
int Derek::Button_debounced::init()
{
	reset();
	
	button_pin = 0;
	last_button_state = 0;
	debounce_response = 0;
	last_debounce_instant = 0;
	debounce_delay = 0;

	return 1;
}

///Init copy initializer
int Derek::Button_debounced::init(Button_debounced &x)
{
	reset();

	button_pin = x.button_pin;
	debounce_delay = x.debounce_delay;

	last_button_state = LOW;
	debounce_response = LOW;
	last_debounce_instant = 0;

	///Setting the pin in INPUT mode
	pinMode(button_pin, INPUT);

	return 1;
}

///Init manual initializer
int Derek::Button_debounced::init(byte b_pin,unsigned long d_delay)
{
	reset();
	
	button_pin = b_pin;
	debounce_delay = d_delay;
	
	last_button_state = LOW;
	debounce_response = LOW;
	last_debounce_instant = 0;

	///Setting the pin in INPUT mode
	pinMode(button_pin, INPUT);
	
	return 1;
}

///Reset function
int Derek::Button_debounced::reset()
{
	button_pin = 0;
	last_button_state = 0;
	debounce_response = 0;
	last_debounce_instant = 0;
	debounce_delay = 0;

	return 1;
}

///Pointers nullifier function
int Derek::Button_debounced::nullifier_pointers()
{

	return 1;
}

///Function to capture the state of the button
boolean Derek::Button_debounced::acquisition_button_state()
{
	last_debounce_instant = 0;
	debounce_response = 0;

	///Counter of debounce responses
	unsigned int counter_responses = 0;

	///The counter can be increased using the debounce response because the boolean type has HIGH = 1 and LOW = 0
	for (int i = 0; i<CHECK_CYCLES; i++)
	{
		counter_responses = counter_responses + debounce();
	}

	if(counter_responses > DECISION_CYCLES)
	{
		return HIGH;
	}
	else
	{
		return LOW;
	}
}

///Function to implement the debounce algorithm
boolean Derek::Button_debounced::debounce()
{
	///Reading the input associated to the button
	boolean reading = digitalRead(button_pin);
	
	///If the actual reading is different from the last reading, save the instant
	if(reading != last_button_state)
		last_debounce_instant = millis();
	
	///If the time between now and the last instant saved is greater then the necessary delay to consider the signal as stable...
	if(millis() - last_debounce_instant >= debounce_delay)
		///...save the actual reading as the response of the debounce function
		debounce_response = reading;
	
	last_button_state = reading;
	
	return debounce_response;
}