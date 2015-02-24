
/**
	@file Scanner_ultrasonic.cpp

	@brief Implementation file for class Scanner_ultrasonic
*/

///Including the header file associated
#include "Scanner_ultrasonic.h"

///Implementations

///Default constructor
Derek::Scanner_ultrasonic::Scanner_ultrasonic()
{
	nullifier_pointers();
	init();
}

///Copy constructor
Derek::Scanner_ultrasonic::Scanner_ultrasonic(Scanner_ultrasonic &x)
{
	nullifier_pointers();
	init(x);
}

///Manual constructor
Derek::Scanner_ultrasonic::Scanner_ultrasonic(Stepper &sc_stepper,Button_debounced &sc_button,DistanceSensorDuo &sc_sensor)
{
	nullifier_pointers();
	init(sc_stepper,sc_button,sc_sensor);
}

///Destructor
Derek::Scanner_ultrasonic::~Scanner_ultrasonic()
{
	reset();
}

///Operator = overloading
int Derek::Scanner_ultrasonic::operator =(Scanner_ultrasonic &x)
{
	// /*
	reset();
	// */

	init(x);

	return 1;
}

///Init default initializer
int Derek::Scanner_ultrasonic::init()
{
	reset();

	return 1;
}

///Init copy initializer
int Derek::Scanner_ultrasonic::init(Scanner_ultrasonic &x)
{
	reset();
	
	scanner_stepper = new Stepper(*x.scanner_stepper);
	scanner_button = new Button_debounced(*x.scanner_button);
	scanner_sensor = new DistanceSensorDuo(*x.scanner_sensor);
	
	quick_scan_position =  0;

	return 1;
}

///Init manual initializer
int Derek::Scanner_ultrasonic::init(Stepper &sc_stepper,Button_debounced &sc_button,DistanceSensorDuo &sc_sensor)
{
	reset();
	
	scanner_stepper = new Stepper(sc_stepper);
	scanner_button = new Button_debounced(sc_button);
	scanner_sensor = new DistanceSensorDuo(sc_sensor);
	
	quick_scan_position =  0;
	
	return 1;
}

///Reset functions
int Derek::Scanner_ultrasonic::reset()
{
	if(scanner_stepper)
	{
		delete scanner_stepper;
	}
	
	if(scanner_button)
	{
		delete scanner_button;
	}
	
	if(scanner_sensor)
	{
		delete scanner_sensor;
	}
	
	nullifier_pointers();
	
	return 1;
}

///Pointers nullifier function
int Derek::Scanner_ultrasonic::nullifier_pointers()
{
	scanner_stepper = 0;
	scanner_button = 0;
	scanner_sensor = 0;
	
	return 1;
}

///Start up function
int Derek::Scanner_ultrasonic::start_up(int button_direction_distance,unsigned int stepper_speed)
{
	scanner_stepper->setSpeed(stepper_speed);
	
	///If the value is equal to zero , then the function does not work and returns zero to warn the user
	if (button_direction_distance == 0)
	{
		return 0;
	}

	///If the sign is positive...
	if (button_direction_distance > 0)
	{
		///...the motor rotates until it finds the button...
		while (scanner_button->acquisition_button_state() == LOW)
		{
			scanner_stepper->step(1);
		}
	}
	///...else the direction is inverted.
	else
	{
		while (scanner_button->acquisition_button_state() == LOW)
		{
			scanner_stepper->step(-1);
		}
	}


	///And then go to the starting point.
	scanner_stepper->step(-button_direction_distance);

	///Success
	return 1;
}

int Derek::Scanner_ultrasonic::rotation(unsigned int quick_scan_distance,unsigned int quick_scan_speed)
{
	
	///Initializing stepper speed
	scanner_stepper->setSpeed(quick_scan_speed);
		
	///CASE A
	if(quick_scan_position ==  0)
	{
		scanner_stepper->step(quick_scan_distance);
		quick_scan_position =  1;
		return 1;
	}
		
	///CASE B
	if(quick_scan_position ==  1)
	{
		scanner_stepper->step(-quick_scan_distance);
		quick_scan_position =  2;
		return 1;
	}
		
	///CASE C
	if(quick_scan_position ==  2)
	{
		scanner_stepper->step(-quick_scan_distance);
		quick_scan_position =  3;
		return 1;
	}
	///CASE D
	if (quick_scan_position == 3)
	{
		scanner_stepper->step(quick_scan_distance);
		quick_scan_position = 0;
		return 1;
	}
	

	//return 1;
}

///Quick scan implementation
int Derek::Scanner_ultrasonic::quick_scan(unsigned int number_scans)
{
	
	number_scans = number_scans * 4;
	unsigned int a[number_scans];

	for (int i = 0; i < number_scans; i++)
	{
		a[i] = scanner_sensor->getDistance_fixed();
		rotation(QUICK_SCAN_DISTANCE, QUICK_SCAN_SPEED);
		delay(WAITING_TIME_SENSOR);

	}
	
	for (int i = 1; i < number_scans; i++)
	{
		if (a[0]>a[i])
		{
			a[0] = a[i];
		}
	}
	
	///Returning the min value
	return a[0];
}

///Full scan
unsigned int Derek::Scanner_ultrasonic::full_scan(byte sectors,unsigned int points_per_sector,unsigned int scan_lenght,unsigned int st_speed,unsigned int *p_results)
{
	///Results of the average of each sector
	unsigned int a[sectors];
	///Total points
	unsigned int tot_points = sectors * (points_per_sector - 1) + 1;
	///Values of acquisitions
	unsigned int b[tot_points];
	///Distance between points
	unsigned int distance_between_points = scan_lenght / tot_points;
	///Distance error
	unsigned int d_error = scan_lenght - (distance_between_points*tot_points);

	///Initializing to 0 to allow the automatic incrementation
	for (int i = 0; i < sectors; i++)
	{
		a[i] = 0;
	}

	///Setting the stepper speed for the initialization
	scanner_stepper->setSpeed(FULL_SCAN_SPEED_INITIALIZING);
	
	///Rotation to start the scan
	scanner_stepper->step(-(scan_lenght / 2));
	
	///Setting the stepper speed with a value chosen from the user
	scanner_stepper->setSpeed(st_speed);

	///Waiting for the end of the vibrations caused by the movement
	delay(WAITING_TIME_SENSOR);


	for (int i = 0; i < tot_points; i++)
	{
		///Getting the distance that the sensor has detected
		b[i] = scanner_sensor->getDistance_fixed();
		///Rotation to prepare the next scan
		scanner_stepper->step(distance_between_points);

		///Distributing the correction between the acquisitions
		if (d_error != 0)
		{
			scanner_stepper->step(1);
			d_error--;
		}

		///Waiting (100 ms) to ensure the correct functioning of the sensor HC-SR04
		delay(WAITING_TIME_SENSOR);
	}

	///Setting the speed to return to the starting point
	scanner_stepper->setSpeed(FULL_SCAN_SPEED_INITIALIZING);
	///Returning to the starting point
	scanner_stepper->step(-(scan_lenght / 2));

	///Correcting the returning in case the error was not corrected completely
	scanner_stepper->step(d_error);

	for (int i = 0; i < sectors; i++)
	{
		///Making the sum of the pints for each sector
		for (int j = 0; j < points_per_sector; j++)
		{
			a[i] = a[i] + b[(j + (points_per_sector - 1) * i)];
		}

		///Calculating the average value for each sector
		a[i] = a[i] / points_per_sector;

		///Writing on the array using the pointer getted as input of the function
		*p_results = a[i];
		p_results++;
	}

	return 1;
}
