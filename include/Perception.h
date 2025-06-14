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
// #include "data_structure.h"

#include <chrono>
#include <filesystem>
#include <iostream>
#include <string>
#include <thread>
#include <mutex>
#include <future>

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
        Perception();
        ~Perception();

        // Parametrized Constructor
        // TODO: Update with correct initialization variables. 
        Perception(string &parentFolderPath);
        // Copy Constructor
        Perception(const Perception &perception);
        // Copy Assignment operator
        Perception &operator=(const Perception &perception);
        // Move constructor
        Perception(Perception &&perception);
        // move assignment operator
        Perception &operator=(Perception &&perception);

        // Member functions
        const bool assertValidInput();
        void init();
        std::set<std::filesystem::path> sortFiles(string &filePath);
        void processCameraData(std::set<std::filesystem::path> &cameraFilesPath);
        void processLidarData(std::set<std::filesystem::path> &lidarFilesPath);
        void processRadarData();
        void processEgoData();
        void processSensorFusion(); 
        void logSensorData();

    private:
        CameraProcessing::Camera *leftCameraObject;
        CameraProcessing::Camera *rightCameraObject;
        LidarProcessing::Lidar<pcl::PointXYZI> *lidarObject;
        Calibration *calibrationObject;
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

        string leftImageFolderPath {};
        string rightImageFolderPath {};
        string lidarFolderPath {};
        // Move all of this into calibration class
        
};
} // namespace Perception
#endif // PERCEPTION_H