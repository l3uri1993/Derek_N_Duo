
/**
	@file Scanner_driver.cpp

	@brief Implementation file for class Scanner_driver
*/

///Including the header file associated
#include "Scanner_driver.h"

///Implementations

///Default constructor
Derek::Scanner_driver::Scanner_driver()
{
	nullifier_pointers();
	init();
}

///Copy constructor
Derek::Scanner_driver::Scanner_driver(Scanner_driver &x)
{
	nullifier_pointers();
	init(x);
}

///Destructor
Derek::Scanner_driver::~Scanner_driver()
{
	reset();
}

///Operator = overloading
int Derek::Scanner_driver::operator =(Scanner_driver &x)
{
	// /*
	reset();
	// */

	init(x);

	return 1;
}

///Init default initializer
int Derek::Scanner_driver::init()
{
	reset();

	return 1;
}

///Init copy initializer
int Derek::Scanner_driver::init(Scanner_driver &x)
{
	reset();

	return 1;
}

///Reset functions
int Derek::Scanner_driver::reset()
{

	return 1;
}

///Pointers nullifier function
int Derek::Scanner_driver::nullifier_pointers()
{

	return 1;
}