
/**
	@file Button_driver.cpp

	@brief Implementation file for class Button_driver
*/

///Including the header file associated
#include "Button_driver.h"

///Implementations

///Default constructor
Derek::Button_driver::Button_driver()
{
	nullifier_pointers();
	init();
}

///Copy constructor
Derek::Button_driver::Button_driver(Button_driver &x)
{
	nullifier_pointers();
	init(x);
}

///Destructor
Derek::Button_driver::~Button_driver()
{
	reset();
}

///Operator = overloading
int Derek::Button_driver::operator =(Button_driver &x)
{
	// /*
	reset();
	// */

	init(x);

	return 1;
}

///Init default initializer
int Derek::Button_driver::init()
{
	reset();

	return 1;
}

///Init copy initializer
int Derek::Button_driver::init(Button_driver &x)
{
	reset();

	return 1;
}

///Reset functions
int Derek::Button_driver::reset()
{

	return 1;
}

///Pointers nullifier function
int Derek::Button_driver::nullifier_pointers()
{

	return 1;
}