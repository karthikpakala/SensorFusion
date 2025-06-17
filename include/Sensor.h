// This class serves as an interface slass for all the sensors that are to be integrated into this project. 
// This will be used to create other classes like Camera, Lidar and Radar classes that will inherit from this class. 

#ifndef SENSOR_H
#define SENSOR_H
#include <iostream>
#include <string>
#include <vector>     

#include "DataQueue.h"

namespace Perception
{
class Sensor
{
    public:
        Sensor(); // Default constructor
        Sensor(const Sensor &sensorObject);
        Sensor &operator=(const Sensor &sensorObject); // Copy Assignment 
        Sensor(Sensor &&sensorObject); // Move Constructor
        Sensor &operator=(Sensor &&sensorObject); // Move Assignment
        virtual ~Sensor(); // virtual destructor

        // Member Functions
        virtual void init() = 0; // initialize the sensor.

    private:
        int dataBufferSize; // size of the data buffer

}; // Sensor Class
} // namespace Perception 

#endif