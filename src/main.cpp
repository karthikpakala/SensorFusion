#include <iostream>
#include <filesystem>
#include "Lidar.h"
#include "Radar.h"
#include "Camera.h"
#include "Calibration.h"

using namespace std;


int main(int argv, char **argc)
{
    // misc
    //double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    //vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results

    // Initialize calibration object
    Calibration calibration;

    // calibration.initializeMatrices();

    // Data file path definitions.
    string baseDataFolderPath = "../KITTI-data";
    string pclDataFolderPath = "/velodyne_points/data/";
    string imageDataFolderPath = "/image_02/data/";
    string fileNamePre = "000000";
    string pclFileType = ".bin";
    string imageFileType = ".png";

    string fullPCLFolderPath = baseDataFolderPath + pclDataFolderPath;
    string fullImageFolderPath = baseDataFolderPath + imageDataFolderPath;

    uint16_t imageFileCount = 0;
    uint16_t pclFileCount = 0;

    // Initiate Lidar points vector
    vector<LidarPoint> lidarPoints;

    // Initiate ac image
    cv::Mat inputImage;

    // PCL File counter
    for(auto& file : filesystem::directory_iterator(fullPCLFolderPath))
    {
        ++pclFileCount;
    }
    
    // Image File Counter
    for(auto& file : filesystem::directory_iterator(fullImageFolderPath))
    {
        ++imageFileCount;
    }

    // This Check assumes that the Camera and Lidar data was collected synchronously at the same frequency and are being used accordingly. 
    // If a different association technique(ex: assiciating every other camerra frame with Lidar frame) is to be used, this logic needs to change.
    if(!(imageFileCount == pclFileCount))
    {
        std::cerr << "Number of image files is not Equal to number of pcl files" << "\n" << "PCL File Count = " << pclFileCount << "\n" << "Image File Count" << imageFileCount << "\n" << endl;
        return 0;
    }
    else
    {     
        // Load Lidatr data into buffer. 
        for(auto& file : filesystem::directory_iterator(fullPCLFolderPath))
        {            
            Lidar lidar;
            lidar.readPCLDataFile(lidarPoints, file.path());
            cout << "Lidar PCD size = " << lidarPoints.size() << endl;

            // Write code to process lidar points
            //lidar.cropLidarPoints(lidarPoints);
            cout << "Lidar Points after cropping = " << lidarPoints.size() << endl;
        }

        // Load input image into the buffer. 
        for(auto& file : filesystem::directory_iterator(fullImageFolderPath))
        {
            inputImage = cv::imread(file.path());
            //cout << "Camera image size = " << inputImage.size() << endl;
            cv::imshow("input Image", inputImage);
            cv::waitKey(100);
            
        }
    }
}