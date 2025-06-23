#include "Perception.h"

using namespace Perception;

// Initilaize the static instance to nullptr
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
    std::cout << "Default Perception constructor called" << std::endl;
    // Initialize image Left Structure
    // inputDataStructure->imageStructLeft.image = cv::Mat::zeros(1242, 375, CV_8UC3); // Initialize with zeroes
    //inputDataStructure->imageStructLeft.image = cv::Mat::zeros(cv::Size(1242, 375), CV_8UC1);

    inputDataStructure->imageStructLeft.keyPoints.clear(); // Clear Key Points
    // inputDataStructure->imageStructLeft.descriptors = cv::Mat::zeros(0, 0, CV_32F); // Initialize with zeroes
    // inputDataStructure->imageStructLeft.descriptors = cv::Mat::zeros(cv::Size(1242, 375), CV_8UC1);

    inputDataStructure->imageStructLeft.boundingBoxes.clear(); // Clear Bounding Boxes

    // Initialize image Right structure
    // inputDataStructure->imageStructRight.image = cv::Mat::zeros(1242, 375, CV_8UC3); // Initialize with zeroes
    // inputDataStructure->imageStructRight.image = cv::Mat::zeros(cv::Size(1242, 375), CV_8UC1);
    inputDataStructure->imageStructRight.keyPoints.clear(); // Clear Key Points
    // inputDataStructure->imageStructRight.descriptors = cv::Mat::zeros(0, 0, CV_32F); // Initialize with zeroes
    // inputDataStructure->imageStructRight.descriptors = cv::Mat::zeros(cv::Size(1242, 375), CV_8UC1);
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
    std::cout << "Delegated Perception constructor called" << std::endl;

    // create folder paths
    leftImageFolderPath = parentFolderPath + imageLeftFilePath;
    rightImageFolderPath = parentFolderPath + imageRightFilePath;
    lidarFolderPath = parentFolderPath + lidarFilePath;
    init();
}

// Parametrized constructor for different image resolution.
Perception::Perception::Perception(std::string &parentFolderPath, int &imageWidth, int &imageHeight) : 
                                    Perception(parentFolderPath)
{
    // Initialize data objects
    std::cout << "Parametrized Perception constructor called for different image resolution" << std::endl;

    // Set input image size to the appropriate values. 
    inputDataStructure->imageStructLeft.image = 
            cv::Mat::zeros(imageWidth, imageHeight, CV_8UC3); // Initialize with input camera resolution.
    inputDataStructure->imageStructRight.image =
            cv::Mat::zeros(imageWidth, imageHeight, CV_8UC3); // Initialize with input camera resolution.
    init();
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
    delete inputDataStructure;

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

    // Initialize promises to get previous keypoints, descriptors and matches for left camera object.
    std::promise<std::vector<cv::KeyPoint>> prevKeyPointsPromiseLeft {};
    std::promise<cv::Mat> prevDescriptorsPromiseLeft {};
    std::promise<std::vector<cv::DMatch>> matchesPromiseLeft{};

    // Initialize futures to get previous keypoints, descriptors and matches for left camera object.
    std::future<std::vector<cv::KeyPoint>> prevKeyPointsFutureLeft = prevKeyPointsPromiseLeft.get_future();
    std::future<cv::Mat> prevDescriptorsFutureLeft = prevDescriptorsPromiseLeft.get_future();
    std::future<std::vector<cv::DMatch>> matchesFutureLeft = matchesPromiseLeft.get_future();


    // Initialize promises to get previous keypoints, descriptors and matches for right camera object.
    std::promise<std::vector<cv::KeyPoint>> prevKeyPointsPromiseRight {};
    std::promise<cv::Mat> prevDescriptorsPromiseRight {};
    std::promise<std::vector<cv::DMatch>> matchesPromiseRight{};

    // Initialize futures to get previous keypoints, descriptors and matches for right camera object.
    std::future<std::vector<cv::KeyPoint>> prevKeyPointsFutureRight = prevKeyPointsPromiseRight.get_future();
    std::future<cv::Mat> prevDescriptorsFutureRight = prevDescriptorsPromiseRight.get_future();
    std::future<std::vector<cv::DMatch>> matchesFutureRight = matchesPromiseRight.get_future();

    uint16_t count = 0; // counter for camera files. 
    // While loop to process each of the files in order. 
    while(cameraLeftIterator != sortedCameraFilesLeft.end() && cameraRightIterator != sortedCameraFilesRight.end() 
           && lidarIterator != sortedLidarFiles.end())
    {
        // Set Left and Right Camera images
        inputDataStructure->imageStructLeft.image = cv::imread((*cameraLeftIterator).string());
        inputDataStructure->imageStructRight.image = cv::imread((*cameraRightIterator).string());

        // Create a viewer object and initilaize the viewer.
        pcl::visualization::PCLVisualizer::Ptr viewer (new pcl::visualization::PCLVisualizer("PCL Viewer"));

        // Disable global warning display for the viewer.
        viewer->getRenderWindow()->GlobalWarningDisplayOff(); 

        while(!viewer->wasStopped())
        {
            std::cout << "*************** Start of Lidar and Camera Processing **************" << std::endl;
          
            // Create a camera angle object and initialize the camera. 
            Tooling::CameraAngle cameraAngle = Tooling::CameraAngle::XY;

            // Initialize the camera angle for PCL viewer.
            toolsObject->initCamera(cameraAngle, viewer);
            viewer->removeAllPointClouds();
            viewer->removeAllShapes();
            
            // Create a image clone for visualization. 
            cv::Mat leftImageClone = inputDataStructure->imageStructLeft.image.clone();
            cv::Mat rightImageClone = inputDataStructure->imageStructRight.image.clone();

            // Initialize the camera and lidar onjects on individual threads. 
            // Left Camera thread
            std::thread cameraLeftThread = std::thread(&CameraProcessing::Camera::cameraProcessing, leftCameraObject,
                                                   std::ref(inputDataStructure->imageStructLeft.image),
                                                   std::ref(detectorType),
                                                   std::ref(descriptorType),
                                                   std::ref(selectorType),
                                                   std::ref(matcherType),
                                                   std::ref(inputDataStructure->imageStructLeft.keyPoints),
                                                   std::ref(inputDataStructure->imageStructLeft.descriptors),
                                                   std::ref(inputDataStructure->imageStructLeft.prevKeyPoints),
                                                   std::move(prevKeyPointsPromiseLeft),
                                                   std::ref(inputDataStructure->imageStructLeft.prevDescriptors),
                                                   std::move(prevDescriptorsPromiseLeft),
                                                   std::ref(inputDataStructure->imageStructLeft.keyPointMatches),
                                                   std::move(matchesPromiseLeft),
                                                   std::ref(matchDescriptorsType),
                                                   std::ref(count),
                                                   std::ref(modelWeightsPath),
                                                   std::ref(modelClassesPath),
                                                   std::ref(modelConfigurationPath));
            
            // Capture previous keypoints, descriptors and matches for left camera. 
            inputDataStructure->imageStructLeft.prevKeyPoints = prevKeyPointsFutureLeft.get(); // Get previous keypoints for left camera
            inputDataStructure->imageStructLeft.prevDescriptors = prevDescriptorsFutureLeft.get(); // Get previous descriptors for left camera
            inputDataStructure->imageStructLeft.keyPointMatches = matchesFutureLeft.get(); // Get matches for left camera

            // Right Camera thread
            std::thread cameraRightThread = std::thread(&CameraProcessing::Camera::cameraProcessing, rightCameraObject,
                                                   std::ref(inputDataStructure->imageStructRight.image),
                                                   std::ref(detectorType),
                                                   std::ref(descriptorType),
                                                   std::ref(selectorType),
                                                   std::ref(matcherType),
                                                   std::ref(inputDataStructure->imageStructRight.keyPoints),
                                                   std::ref(inputDataStructure->imageStructRight.descriptors),
                                                   std::ref(inputDataStructure->imageStructRight.prevKeyPoints),
                                                   std::move(prevKeyPointsPromiseRight),
                                                   std::ref(inputDataStructure->imageStructRight.prevDescriptors),
                                                   std::move(prevDescriptorsPromiseRight),
                                                   std::ref(inputDataStructure->imageStructRight.keyPointMatches),
                                                   std::move(matchesPromiseRight),
                                                   std::ref(matchDescriptorsType),
                                                   std::ref(count),
                                                   std::ref(modelWeightsPath),
                                                   std::ref(modelClassesPath),
                                                   std::ref(modelConfigurationPath));
            
            // Capture previous keypoints, descriptors and matches for right camera.
            inputDataStructure->imageStructRight.prevKeyPoints = prevKeyPointsFutureRight.get(); // Get previous keypoints for right camera
            inputDataStructure->imageStructRight.prevDescriptors = prevDescriptorsFutureRight.get(); // Get previous descriptors for right camera
            inputDataStructure->imageStructRight.keyPointMatches = matchesFutureRight.get(); // Get matches for right camera

            count++; // Increment the count for camera files processed.
            
            // Lidar processing thread
            std::thread lidarThread = std::thread(&LidarProcessing::Lidar<pcl::PointXYZI>::readPCLDataFile, lidarObject,
                                               (*lidarIterator).string(),
                                               std::ref(viewer));

            cv::drawKeypoints(inputDataStructure->imageStructLeft.image, inputDataStructure->imageStructLeft.keyPoints, 
                                leftImageClone, cv::Scalar(-1,-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            cv::drawKeypoints(inputDataStructure->imageStructRight.image, inputDataStructure->imageStructRight.keyPoints, 
                                rightImageClone, cv::Scalar(-1,-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

            std::string windowNameLeft = "Left Camera Image";
            cv::namedWindow(windowNameLeft, 6);
            imshow(windowNameLeft, leftImageClone);
            cv::waitKey(1);

            std::string windowNameRight = "Right Camera Image";
            cv::namedWindow(windowNameRight, 6);
            imshow(windowNameRight, rightImageClone);
            cv::waitKey(1);

            // Wait for the threads to finish processing.
            cameraLeftThread.join();
            cameraRightThread.join();
            lidarThread.join();

            // Increment the iterators to point to the next files in the sorted list.
            cameraLeftIterator++;
            cameraRightIterator++;
            lidarIterator++;
            std::cout << "*************** End of Lidar and Camera Processing **************" << std::endl;
        }
    }
    // Process each of the files list to be processed in a different thread. 

    // Create individual threads for of the objects to process data. 
    // processCameraData(&sortedCameraFilesLeft);
    // processLidarData(&sortedLidarFiles); 
    // Create separate threads for each of the Camera and Lidar objects.
    // Instantiate each of the Camera and Lidar objects to start processing them. 
}

// void Perception::Perception::processCameraData(int &detectorType, int &descriptorType, int &count, DataStructure::InputStructure::CameraImageStruct &cameraStruct,
//                                                std::string &selectorType, std::string &matcherType, std::string &matchDescriptorType)
// {
//     // Call the camera processing function to process the camera data. 
//     leftCameraObject->cameraProcessing(std::ref(cameraStruct.image),
//                                         std::ref(detectorType),
//                                         std::ref(descriptorType),
//                                         std::ref(selectorType),
//                                         std::ref(matcherType),
//                                         std::ref(cameraStruct.keyPoints),
//                                         std::ref(cameraStruct.descriptors),
//                                         std::ref(cameraStruct.prevKeyPoints),
//                                         std::move(prevKeyPointsPromise),
//                                         std::ref(cameraStruct.prevDescriptors),
//                                         std::move(prevDescriptorsPromise),
//                                         std::ref(cameraStruct.keyPointMatches),
//                                         std::move(matchesPromise),
//                                         std::ref(matchDescriptorType),
//                                         std::ref(count),
//                                         std::ref(modelWeightsPath),
//                                         std::ref(modelClassesPath),
//                                         std::ref(modelConfigurationPath));
// }