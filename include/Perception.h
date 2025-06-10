// This Class is responsible to bring together all the other classes to create a conerent Sensor Fusion module. 
// It will handle the initialization of other classes, manage the data pipeline,and privide a unified interface for sensor fusion operatrions. 
// Some of the key functionaloties of the class include:

// 1. Create buffers (circular buffer) for the sensor data. 
// 2. Initialize parameters for the sensor fusion module.
// 3. Iniitalize and process sensor data within this class. 
// 4. Provide methods for processing sensor data, including reading, processing and storing results.
// 5. Provide methods for accessing processed data and results.

// Methods

#ifndef PERCEPTION_H
#define PERCEPTION_H
#include "Calibration.h"
#include "Camera.h"
#include "Lidar.h"
#include "Radar.h"
#include "Tools.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <future>
namespace Perception {
class Perception {
    public:
        Perception() = default;
        ~Perception() = default;

        Perception(const &Perception); // Copy Constructor
        Perception &operator=(const &Perception); // Copy assignment operator
        Perception (Perception &&); // Move Constructor
        Perception &operator=(Perception &&); // Move assignment operator
        
};
} // namespace Perception
#endif // PERCEPTION_H