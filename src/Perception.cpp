#include "Perception.h"

using namespace Perception;

// using namespace Perception::Tools;

// Default Constructor
// Create the input data structure on the heap, given it contains heavy and complex data variables. 
// Not creating a heap here as the point cloud class has its own heap for the point cloud data. 
// other data variables are images, etc. may not be necessary to create a heap. to be revisited later. 
// Initialize all the data variables to default values.

Perception::Perception::Perception()
{
    // Initialize input data objects

    // Initialize image Left Structure
    inputDataStructure->imageStructLeft.image = cv::Mat::zeros(1242, 375, CV_8UC3); // Initialize with zeroes
    inputDataStructure->imageStructLeft.keyPoints.clear(); // Clear Key Points
    inputDataStructure->imageStructLeft.descriptors = cv::Mat::zeros(0, 0, CV_32F); // Initialize with zeroes
    inputDataStructure->imageStructLeft.boundingBoxes.clear(); // Clear Bounding Boxes

    // Initialize image Right structure
    inputDataStructure->imageStructRight.image = cv::Mat::zeros(1242, 375, CV_8UC3); // Initialize with zeroes
    inputDataStructure->imageStructRight.keyPoints.clear(); // Clear Key Points
    inputDataStructure->imageStructRight.descriptors = cv::Mat::zeros(0, 0, CV_32F); // Initialize with zeroes
    inputDataStructure->imageStructRight.boundingBoxes.clear(); // Clear Bounding Boxes

    // Initialize Lidar Point Cloud
    inputDataStructure->cloud.clear(); // Clear Lidar Point Cloud
    inputDataStructure->segmentedPointCloud.first.clear(); // Clear Segmented Road Surface
    inputDataStructure->segmentedPointCloud.second.clear(); // Clear Segmented Objects
    inputDataStructure->segmentedObjects.clear(); // Clear segmented objects

}

// Parametrized Constructor
// Initialize with corresponding image width and image height.
// Update this constructor to enable more precise handling on the input from the main function.  
Perception::Perception::Perception(string &parentFolderPath)
{
    // Initialize input data objects

    // create folder paths and initialize input data structure.
    leftImageFolderPath = parentFolderPath + imageLeftFilePath;
    rightImageFolderPath = parentFolderPath + imageRightFilePath;
    lidarFolderPath = parentFolderPath + lidarFilePath;

    // assert the validity of input
    // Create camera objects and lidar objects
    // Process data.
    // Set image height and width from the left and right camera objects
    // Change this to use calibration object based image width and height.
    
    //int imageLeftWidth = leftCameraObject->inputImage.rows; // Get the image width from the left camera object
    //int imageLeftHeight = leftCameraObject->inputImage.cols; // Get the image height from the left camera object

    // Initialize image Left Structure
    inputDataStructure->imageStructLeft.image = cv::Mat::zeros(1, 1, CV_8UC3); // Initialize with zeroes
    inputDataStructure->imageStructLeft.keyPoints.clear(); // Clear Key Points
    inputDataStructure->imageStructLeft.descriptors = cv::Mat::zeros(0, 0, CV_32F); // Initialize with zeroes
    inputDataStructure->imageStructLeft.boundingBoxes.clear(); // Clear Bounding Boxes

    //int imageRightWidth = rightCameraObject->inputImage.rows; // Get the image width from the right camera object
    //int imageRightHeight = rightCameraObject->inputImage.cols; // Get the image height from the right camera object

    // Initialize image Right structure
    inputDataStructure->imageStructRight.image = cv::Mat::zeros(1, 1, CV_8UC3); // Initialize with zeroes
    inputDataStructure->imageStructRight.keyPoints.clear(); // Clear Key Points
    inputDataStructure->imageStructRight.descriptors = cv::Mat::zeros(0, 0, CV_32F); // Initialize with zeroes
    inputDataStructure->imageStructRight.boundingBoxes.clear(); // Clear Bounding Boxes

    // Initialize Lidar Point Cloud
    inputDataStructure->cloud.clear(); // Clear Lidar Point Cloud
    inputDataStructure->segmentedPointCloud.first.clear(); // Clear Segmented Road Surface
    inputDataStructure->segmentedPointCloud.second.clear(); // Clear Segmented Objects
    inputDataStructure->segmentedObjects.clear(); // Clear segmented objects

    init(); // Initialize the perception object


}

Perception::Perception::~Perception()
{

}

Perception::Perception::Perception(const Perception &perception)
{
    // Copy constructor implementation
    if (this != &perception)
    {

    }

}

const bool Perception::Perception::assertValidInput()
{
    bool isValid = true;
    int imageLeftFileCount = 0;
    int imageRightFileCount = 0;
    int lidarFileCount = 0;
    for(auto &file : std::filesystem::directory_iterator(leftImageFolderPath))
    {   
        imageLeftFileCount++;
    }
    for(auto &file : std::filesystem::directory_iterator(rightImageFolderPath))
    {
        imageRightFileCount++;
    }
    for(auto &file : std::filesystem::directory_iterator(lidarFolderPath))
    {
        lidarFileCount++;
    }

    if(imageLeftFileCount != imageRightFileCount || imageLeftFileCount != lidarFileCount)
    {
        std::cerr << "Number of image files is not Equal to number of pcl files"
                  << "\n"
                  << "PCL File Count = " << lidarFileCount << "\n"
                  << "Right Image File Count" << imageRightFileCount << "\n"
                  << "Left Image File Count" << imageLeftFileCount << "\n"
                  << endl;
        isValid = false;
    }
    return isValid;
}

std::set<std::filesystem::path> Perception::Perception::sortFiles(string &folderPath)
{
    std::set<std::filesystem::path> sortedFiles {};
    for(auto &file : std::filesystem::directory_iterator(folderPath))
    {
        sortedFiles.insert(file.path());
    }
    return sortedFiles;
}

// 1. Sort the files in folder.  
// 2. create the list of files in the list of files.
// 3. Process each of the files in the list of files. 
// 4. Process each of the files for lidar and camera images. 
void Perception::Perception::init()
{
    // assert the validity of input
    if(!assertValidInput())
    {
        std::cout << "Invalid Input!!" << std::endl;
        return;
    }
    // Sort the files in the given folders. 
    std::set<std::filesystem::path> sortedCameraFilesLeft = sortFiles(leftImageFolderPath);
    std::set<std::filesystem::path> sortedCameraFilesRight = sortFiles(rightImageFolderPath);
    std::set<std::filesystem::path> sortedLidarFiles = sortFiles(lidarFolderPath);

    // Process each of the files list to be processed in a different thread. 
    std::mutex perceptionMutex;
    std::lock_guard<std::mutex> perceptionLock(perceptionMutex);
    // Create individual threads for of the objects to process data. 
    // processCameraData(&sortedCameraFilesLeft);
    // processLidarData(&sortedLidarFiles); 
    // Create separate threads for each of the Camera and Lidar objects.
    // Instantiate each of the Camera and Lidar objects to start processing them. 
}

void Perception::Perception::processCameraData(std::set<std::filesystem::path> &cameraFilesPath)
{
    // Initiate a new thread to start processing the camera data. 
    //std::thread cameraThread = std::thread(&Perception::CameraProcessing::Camera::cameraProcessing, leftCameraObject)

}