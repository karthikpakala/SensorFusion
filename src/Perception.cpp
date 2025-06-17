#include "Perception.h"

using namespace Perception;

Perception::Perception* Perception::Perception::instance = nullptr;
// using namespace Perception::Tools;

// Default Constructor
// Create the input data structure on the heap, given it contains heavy and complex data variables. 
// Not creating a heap here as the point cloud class has its own heap for the point cloud data. 
// other data variables are images, etc. may not be necessary to create a heap. to be revisited later. 
// Initialize all the data variables to default values.

Perception::Perception::Perception()
{
    // Initialize input data objects to default values.
    std::cout << "Default constructor called" << std::endl;
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
Perception::Perception::Perception(string &parentFolderPath) : Perception()
{
    // Initialize input data objects
    std::cout << "Delegated constructor calling" << std::endl;

    // create folder paths
    leftImageFolderPath = parentFolderPath + imageLeftFilePath;
    rightImageFolderPath = parentFolderPath + imageRightFilePath;
    lidarFolderPath = parentFolderPath + lidarFilePath;
}

// Parametrized constructor for different image resolution.
Perception::Perception::Perception(std::string &parentFolderPath, int &imageWidth, int &imageHeight) : 
                                    Perception(parentFolderPath)
{
    // Initialize data objects
    std::cout << "Parametrized constructor called for different image resolution" << std::endl;

    // Set input image size to the appropriate values. 
    inputDataStructure->imageStructLeft.image = 
            cv::Mat::zeros(imageWidth, imageHeight, CV_8UC3); // Initialize with input camera resolution.
    inputDataStructure->imageStructRight.image =
            cv::Mat::zeros(imageWidth, imageHeight, CV_8UC3); // Initialize with input camera resolution.
}

// Destructor
Perception::Perception::~Perception()
{
    cout << 
        "Perception Destructor Called - Killing Left Camera, Right Camera, Lidar and Calibration Objects" 
        << endl;
    // kill all objects by calling their respective destructors. 
    delete leftCameraObject;
    delete rightCameraObject;
    delete lidarObject;
    delete calibrationObject;

}

// Singleton Instance Getter
Perception::Perception* Perception::Perception::getInstance()
{
    if(instance == nullptr)
    {
        instance = new Perception();
    }
    return instance;
}

// Parametrized Singleton Instance Getter
Perception::Perception* Perception::Perception::getInstance(string &parentFolderPath)
{
    if(instance == nullptr)
    {
        instance = new Perception(parentFolderPath);
    }
    return instance;
}

// Parametrized Singleton Instance Getter for a different image resolution 
Perception::Perception* Perception::Perception::getInstance(string &parentFolderPath, int &imageWidth, int &imageHeight)
{
    if(instance == nullptr)
    {
        instance = new Perception(parentFolderPath, imageWidth, imageHeight);
    }
    return instance;
}

// Assert validity of input data
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

    if(imageLeftFileCount != imageRightFileCount || imageLeftFileCount != lidarFileCount || imageRightFileCount != lidarFileCount)
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

// Sort files in the given folder path. 
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

    auto cameraLeftIterator = sortedCameraFilesLeft.begin();
    auto cameraRightIterator = sortedCameraFilesRight.begin();
    auto lidarIterator = sortedLidarFiles.begin();
    
    int detectorType {};
    int descriptorType {};

    string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
    string matchDescriptorsType = "DES_BINARY"; // DES_BINARY, DES_HOG
    string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN // TODO: Fix SEL_NN algorithm - Matches coming out to be 0
    
    // Initialize camera parameters
    leftCameraObject->init(detectorType, descriptorType);
    rightCameraObject->init(detectorType, descriptorType);

    while(cameraLeftIterator != sortedCameraFilesLeft.end() && cameraRightIterator != sortedCameraFilesRight.end() 
           && lidarIterator != sortedLidarFiles.end())
    {
        // Create a thread for each of the Cameras and Lidar processing. 
        std::thread cameraLeftThread(&Perception::Perception::processCameraData, leftCameraObject,
                                                  std::ref(cameraLeftIterator->string()), 
                                                  std::ref(detectorType), 
                                                  std::ref(descriptorType), 
                                                  std::ref(selectorType), 
                                                  std::ref(matcherType), 
                                                  std::ref(inputDataStructure->imageStructLeft.keyPoints), 
                                                  std::ref(inputDataStructure->imageStructLeft.descriptors), 
                                                  std::ref(prevKeyPoints), 
                                                  std::move(prevKeyPointsPromise), 
                                                  std::ref(prevDescriptors), 
                                                  std::move(prevDescriptorsPromise), 
                                                  std::ref(matches),
                                                  std::move(matchesPromise), 
                                                  std::ref(matchDescriptorsType), 
                                                  std::ref(cameraCount),
                                                  std::ref(modelWeightsPath),
                                                  std::ref(modelClassesPath),
                                                  std::ref(modelConfigurationPath));
    }
    // Process each of the files list to be processed in a different thread. 

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