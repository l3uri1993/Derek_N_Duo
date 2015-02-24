
/**
	@file Scanner_driver.h

	@brief Header file for class Scanner_driver
*/

///Including Arduino library
#include <Arduino.h>

///Protection from redefinition error
#ifndef Scanner_driver_DEF
#define Scanner_driver_DEF

/**
	@class Scanner_driver

	@brief Scanner device driver definition for the Derek robot
*/

namespace Derek
{

	class Scanner_driver
	{

	private:

	protected:

	public:

		///Constructors
		Scanner_driver();
		Scanner_driver(Scanner_driver &x);

		///Destructor
		virtual ~Scanner_driver();

		///Operator = overloading
		int operator =(Scanner_driver &x);

		///Init functions
		int init();
		int init(Scanner_driver &x);

		///Reset functions
		int reset();

		///Pointers nullifier function
		int nullifier_pointers();
	};
};

#endif