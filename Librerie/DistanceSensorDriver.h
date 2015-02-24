/**
 * @file distance_sensor.h
 * @brief distance sensor driver definition for the Derek robot.
 */

#ifndef _DISTANCE_SENSOR_DEF
#define _DISTANCE_SENSOR_DEF

namespace Derek
{
    class DistanceSensorDriver
    {
    public:
        /**
          * @brief Class constructors.
          * @param distance The maximum distance in centimeters that needs to be tracked.
          */
		DistanceSensorDriver();
        DistanceSensorDriver(unsigned int distance);
		~DistanceSensorDriver();
        
        /**
         * @brief Return the distance to the nearest obstacle in centimeters.
         * @return the distance to the closest object in centimeters 
         *   or maxDistance if no object was detected.
         */
        virtual unsigned int getDistance() = 0;					//interface, implemented in the specific class of the device used.
      
    protected:
        unsigned int maxDistance;
    };
};

#endif