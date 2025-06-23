// This Class is responsible to bring together all the other classes to create a conerent Sensor Fusion module. 
// It will handle the initialization of other classes, manage the data pipeline,and privide a unified interface for sensor fusion operatrions. 
// Some of the key functionaloties of the class include:

// 1. Create buffers (circular buffer) for the sensor data. 
// 2. Initialize parameters for the sensor fusion module.
// 3. Iniitalize and process sensor data within this class. 
// 4. Provide methods for processing sensor data, including reading, processing and storing results.
// 5. Provide methods for accessing processed data and results.

// Methods

// Use Facade Design pattern to make sure this class only serves as a framework upon which other class 
// objects are built and used. This enables a clearly defined interface to manage sensor fusion modules. 
// the class objects created within this class are all based on the class objects that are inherited from
// a different Sensor class that defines the interface for other new classes (Camera, Lidar, Radar) to be 
// created for different sensors.  
#ifndef PERCEPTION_H
#define PERCEPTION_H

#include "Calibration.h"
#include "Camera.h"
#include "Lidar.h"
#include "Radar.h"
#include "Tools.h"
// #include "data_structure.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
// #include <thread>
// #include <mutex>
// #include <future>

using namespace std;
namespace Perception 
{
// singleton class
// 1. Create a default class to initialize all the variables with default values. Also, create a singleton class behavior 
// 2. Create a parameterized class constructor to take in different image width and image height values. Also, create a singleton class behavior.(Different camera)
// 3. Create a copy constructor to create a copy of the Perception object. 
// 4. Create a move constructor to move it to the new object.
class Perception 
{
    public:

        // Parametrized Constructor
        // TODO: Update with correct initialization variables. 
        Perception(string &parentFolderPath);
        // constructor for different camera Resolution
        Perception(string &parentFolderPath, int &imageWidth, int &imageHeight);

        // Copy Constructor
        // Delete copy constructor to avoid copying the object.
        Perception(const Perception &perception) = delete; 
        // Copy Assignment operator
        // Delete copy assignment operator to avoid copying the object.
        Perception &operator=(const Perception &perception) = delete; 
        // Move constructor
        // Delete move constructor to avoid moving the object.
        Perception(Perception &&perception) = delete; 
        // move assignment operator
        // Delete move assignment operator to avoid moving the object.
        Perception &operator=(Perception &&perception) = delete; 

        // Destructor
        ~Perception();

        // Member functions
        const bool assertValidInput(); // Assert
        void init();
        inline std::set<std::filesystem::path> sortFiles(string &filePath); // inline to allow for faster processing of sorting operation. 
        void processCameraData(std::set<std::filesystem::path> &cameraFilesPath);
        void processLidarData(std::set<std::filesystem::path> &lidarFilesPath);
        void processRadarData();
        void processEgoData();
        void processSensorFusion(); 
        void logSensorData();

        static Perception* getInstance(); // Returns the class instance. 

        static Perception* getInstance(string &parentFolderPath); // Returns the overloaded class instance. 
        static Perception* getInstance(string &parentFolderPath, int &imageWidth, int &imageHeight); // Returns the overloaded class instance. 
        
        static Perception* instance; // Singleton instance of the class
    private:

        // Making this constructor private to enforce it 
        // not being available for use outside of the class. 
        Perception();

        CameraProcessing::Camera *leftCameraObject;
        CameraProcessing::Camera *rightCameraObject;
        LidarProcessing::Lidar<pcl::PointXYZI> *lidarObject;
        Calibration *calibrationObject;
        Tooling::Tools *toolsObject;
        DataStructure::InputStructure *inputDataStructure;
        // RadarProcessing::Radar *radarObject;

        // Move all of this into calibration class
        string imageLeftFilePath = "/image_01/data/"; // Path to the left camera images
        string imageRightFilePath = "/image_02/data/"; // Path to the right camera images
        string lidarFilePath = "/velodyne_points/data/"; // Path to the lidar point cloud data
        string fileNamePre = "000000";
        string pclFileType = ".bin";
        string imageFileType = ".png";
        string egoFileType = ".txt";


        // Can be removed once the application is updated to use reltime data.
        string leftImageFolderPath {};
        string rightImageFolderPath {};
        string lidarFolderPath {};

        // Path to the perception model configuration files. 
        string modelWeightsPath {};
        string modelClassesPath {};
        string modelConfigurationPath {};

        // Move all of this into calibration class

        //std::mutex perceptionMutex;
        //std::lock_guard<std::mutex> perceptionLock(perceptionMutex);

        
        
};
} // namespace Perception
#endif // PERCEPTION_H