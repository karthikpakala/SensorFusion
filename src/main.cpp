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
#include <atomic>
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/path_traits.hpp>

#include <opencv2/core/mat.hpp>
#include <opencv2/imgproc.hpp>
//#include <pcl-1.13/pcl/impl/point_types.hpp>
#include<pcl/impl/point_types.hpp>

using namespace std;
using namespace Tooling;
using namespace Perception::LidarProcessing;
using namespace Perception::CameraProcessing;
using namespace Perception::DataStructure;

int main(int argv, char **argc) 
{
  int dataBufferSize = 2; // no. of images which are held in memory (ring
                          // buffer) at the same time
  // vector<DataFrame> dataBuffer; // list of data frames which are held in
  // memory at the same time

  // Initialize calibration object
  Calibration calibration;

  // Tools Object
  Tools tools;

  // calibration.initializeMatrices();

  // Data file path definitions.
  #if __linux__ 
    string baseDataFolderPath = "/home/ubuntu/Projects/Data/KITTI-data2"; // File path for linux
    //string baseDataFolderPath = "/home/karthikpakala/Pers-Projects/Data/Kitti-data3"; // Linux HP

    string modelBasePath = "/home/ubuntu/Projects/SensorFusion/model/yolo/"; // File Path for Linux WS
    //string modelBasePath = "/home/karthikpakala/Pers-Projects/SensorFusion/model/yolo/"; // Office Linux
  #else
    string baseDataFolderPath = "/Users/karthikpakala/Projects/Data/KITTI-data3"; // File path for macosx
    string modelBasePath = "/Users/karthikpakala/Projects/SensorFusion/model/yolo/";
  #endif


  string modelWeightsPath = modelBasePath + "yolov3.weights";
  string modelClassesPath = modelBasePath + "coco.names";
  string modelConfigurationPath = modelBasePath + "yolov3.cfg";

  string pclDataFolderPath = "/velodyne_points/data/";
  string imageRightDataFolderPath = "/image_02/data/"; // Right Image from Color Streo Camera
  string imageLeftDataFolderPath = "/image_01/data"; // Left Image from Color Stereo Camera
  string egoFolderPath = "/oxts/data/";
  string fileNamePre = "000000";
  string pclFileType = ".bin";
  string imageFileType = ".png";
  string egoFileType = ".txt";
  
  InputStructure inputDataStructure{};

  string fullPCLFolderPath = baseDataFolderPath + pclDataFolderPath;
  string fullRightImageFolderPath = baseDataFolderPath + imageRightDataFolderPath;
  string fullLeftImageFolderPath = baseDataFolderPath + imageLeftDataFolderPath;
  string fullEgoFolderPath = baseDataFolderPath + egoFolderPath;

  uint16_t imageRightFileCount = 0;
  uint16_t imageLeftFileCount = 0;
  uint16_t pclFileCount = 0;
  uint16_t egoFileCount = 0;
  uint16_t fileCount = 0;

  vector<cv::Mat> imageBuffer;
  int imageBufferSize = 2;

  // PCL File counter
  for (auto &file : std::filesystem::directory_iterator(fullPCLFolderPath)) 
  {
    ++pclFileCount;
  }

  // Image File Counter
  for (auto &file : std::filesystem::directory_iterator(fullRightImageFolderPath)) 
  {
    ++imageRightFileCount;
  }

    // Image File Counter
  for (auto &file : std::filesystem::directory_iterator(fullLeftImageFolderPath)) 
  {
    ++imageLeftFileCount;
  }

    // Ego File Counter
  for (auto &file : std::filesystem::directory_iterator(fullEgoFolderPath)) 
  {
    ++egoFileCount;
  }

  // This Check assumes that the Camera and Lidar data was collected
  // synchronously at the same frequency and are being used accordingly. If a
  // different association technique(ex: assiciating every other camerra frame
  // with Lidar frame) is to be used, this logic needs to change.
  if (imageLeftFileCount != pclFileCount || egoFileCount != imageRightFileCount || egoFileCount != pclFileCount || imageLeftFileCount != imageRightFileCount) 
  {
    std::cerr << "Number of image files is not Equal to number of pcl files"
              << "\n"
              << "PCL File Count = " << pclFileCount << "\n"
              << "Right Image File Count" << imageRightFileCount << "\n"
              << "Left Image File Count" << imageLeftFileCount << "\n"
              << "Ego File Count" << egoFileCount << "\n"
              << endl;
    return 0;
  }
  // Start Sensor Data Processing Loop
  else 
  {

    std::set<std::filesystem::path> sortedPCLFiles;
    // ********************Lidar data files sort******************** //
    for (auto &file :
         std::filesystem::directory_iterator(fullPCLFolderPath))
    {
      sortedPCLFiles.insert(file.path());
    }
    // ********************Lidar Data Sort*************************** //

    // ********************** Camera data files sort **************** //
    std::set<filesystem::path> sortedRightCameraFiles;
    for (auto &file : filesystem::directory_iterator(fullRightImageFolderPath)) 
    {
      sortedRightCameraFiles.insert(file.path());
    }

    std::set<filesystem::path> sortedLeftCameraFiles;
    for (auto &file : filesystem::directory_iterator(fullLeftImageFolderPath)) 
    {
      sortedLeftCameraFiles.insert(file.path());
    }
    // *********************** Camera Data Sort *********************** //

    // *********************** Ego Files Sort ************************* //
    std::set<std::filesystem::path> sortedEgoFiles;
    for (auto &file :
         std::filesystem::directory_iterator(fullEgoFolderPath))
    {
      sortedEgoFiles.insert(file.path());
    }
    // ************************ Ego Files Sort ************************** //


    // Number of CPU cores
    unsigned int nCores =  std::thread::hardware_concurrency();

    // Enable / Disable using Camera / Lidar
    bool useLidar = true;
    bool useCamera = true;
    bool useEgoData = false;

    bool detectKeyPoints = true;
    bool detectObjects = true;
    // RANSAC Segmentation parameters
    int numIterations = 50;
    float distThreshold = 0.359;

    if(useLidar && useCamera)
    {
      fileCount = pclFileCount;
      auto cameraRightIterator = sortedRightCameraFiles.begin();
      auto cameraLeftIterator = sortedLeftCameraFiles.begin();
      auto pclIterator = sortedPCLFiles.begin();
      auto egoIterator = sortedEgoFiles.begin();

      uint16_t cameraCount = 0;
      while(cameraRightIterator != sortedRightCameraFiles.end() && pclIterator != sortedPCLFiles.end() && cameraLeftIterator != sortedLeftCameraFiles.end())
      {
        std::vector<std::future<void>> futures;
        Lidar<pcl::PointXYZI> lidarObject;
        Camera cameraObject;

        // Camera Processing
        // Default Values
        string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
        string matchDescriptorsType = "DES_BINARY"; // DES_BINARY, DES_HOG
        string selectorType = "SEL_KNN";       // SEL_NN, SEL_KNN // TODO: Fix SEL_NN algorithm - Matches coming out to be 0
        int detectorType {};
        int descriptorType {};
        

        // Current Key Points and Descriptors
        std::vector<cv::KeyPoint> keyPoints{};
        cv::Mat descriptors {};

        // Previous Key Points and Descriptors
        std::vector<cv::KeyPoint> prevKeyPoints {};
        cv::Mat prevDescriptors {};

        // Matches from prev and current frames
        std::vector<cv::DMatch> matches {};
        
        // Initialize camera detection parameters
        cameraObject.init(detectorType, descriptorType);

        // Initialize PCL Tools
        Tools *tools;
        pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("3D Viewer"));
        viewer->getRenderWindow()->GlobalWarningDisplayOff();
        while (!viewer->wasStopped()) 
        {
          std::cout << "*************** Start of Lidar and Camera Processing **************" << std::endl;
          CameraAngle cameraAngle = XY;
          tools->initCamera(cameraAngle, viewer);
          viewer->removeAllPointClouds();
          viewer->removeAllShapes();

          std::thread lidarThread = std::thread(&Lidar<pcl::PointXYZI>::readPCLDataFile, &lidarObject, (*pclIterator).string(), std::ref(viewer));
          cv::Mat inputRightImage = cv::imread((*cameraRightIterator).string());
          cv::Mat inputLeftImage = cv::imread((*cameraLeftIterator).string());

          cameraCount++;

          inputDataStructure.imageStructLeft.image = inputLeftImage;
          inputDataStructure.imageStructRight.image = inputRightImage;

          // std::thread cameraThread;
          // std::thread objectDetectionThread;
          // Clone Image for visualization
          cv::Mat visImage = inputRightImage.clone();

          //if(detectKeyPoints)
          //{
            // Right Image Processing
            std::promise<std::vector<cv::KeyPoint>> prevKeyPointsPromise;
            std::promise<cv::Mat> prevDescriptorsPromise;
            std::promise<std::vector<cv::DMatch>> matchesPromise;

            std::future<std::vector<cv::KeyPoint>> prevKeyPointsFuture = prevKeyPointsPromise.get_future();
            std::future<cv::Mat> prevDescriptorsFuture = prevDescriptorsPromise.get_future();
            std::future<std::vector<cv::DMatch>> matchesFuture = matchesPromise.get_future();

            std::thread cameraThread = std::thread(&Perception::CameraProcessing::Camera::cameraProcessing, cameraObject, 
                                                  std::ref(inputRightImage), 
                                                  std::ref(detectorType), 
                                                  std::ref(descriptorType), 
                                                  std::ref(selectorType), 
                                                  std::ref(matcherType), 
                                                  std::ref(keyPoints), 
                                                  std::ref(descriptors), 
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
            // Update previous Key Points and Descriptors - Get data from other thread to use it in next iteration - Pending
            // In Development /////
            prevKeyPoints = prevKeyPointsFuture.get();
            prevDescriptors = prevDescriptorsFuture.get();

            // Visualize Key Point Detection Visualization
            cv::drawKeypoints(inputRightImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
          //}
          // In Development /////
          /*
          //if(detectObjects)
          //{
            vector<Perception::DataStructure::BoundingBox> bBoxes {};
            vector<string> classes{};
            vector<int> classIds{};
            vector<float> confidences{};
            vector<cv::Rect> boundingBoxes {};

            std::promise<vector<Perception::DataStructure::BoundingBox>> bBoxesPromise;
            std::promise<vector<string>> classesPromise;
            std::promise<vector<int>> classIdsPromise;
            std::promise<vector<float>> confidencesPromise;
            std::promise<vector<cv::Rect>> boundingBoxesPromise;

            std::future<vector<Perception::DataStructure::BoundingBox>> bBoxesFuture = bBoxesPromise.get_future();
            std::future<vector<string>> classesFuture = classesPromise.get_future();
            std::future<vector<int>> classIdsFuture = classIdsPromise.get_future();
            std::future<vector<float>> confidencesFuture = confidencesPromise.get_future();
            std::future<vector<cv::Rect>> boundingBoxesFuture = boundingBoxesPromise.get_future();

            std::thread objectDetectionThread = std::thread(&Perception::CameraProcessing::Camera::detectObjects, cameraObject, 
                                                          std::ref(inputRightImage), 
                                                          std::ref(modelWeightsPath),
                                                          std::ref(modelClassesPath),
                                                          std::ref(modelConfigurationPath),
                                                          std::ref(bBoxesPromise),
                                                          std::ref(classesPromise),
                                                          std::ref(classIdsPromise),
                                                          std::ref(confidencesPromise),
                                                          std::ref(boundingBoxesPromise));  

            bBoxes = bBoxesFuture.get();
            classes = classesFuture.get();
            classIds = classIdsFuture.get();
            confidences = confidencesFuture.get();
            boundingBoxes = boundingBoxesFuture.get();

            // Visualize Object Detection
            for(auto it = bBoxes.begin(); it != bBoxes.end(); ++it)
            {

              // Draw Rectangle displaying the boundinh box
              int top, left, width, height;
              top = (*it).roi.y;
              left = (*it).roi.x;
              width = (*it).roi.width;
              height = (*it).roi.height;
              cv::rectangle(visImage, cv::Point(left, top), cv::Point(left+width, top+height), cv::Scalar(0, 255, 0), 2);

              string label = cv::format("%f", (*it).confidence);
              label = classes[((*it).classID)] + ":" + label;

              // Display label at the top of the bounding box
              int baseline;
              cv::Size labelSize = getTextSize(label, cv::FONT_ITALIC, 0.5, 1, &baseline);
              top = max(top, labelSize.height);
              rectangle(visImage, cv::Point(left, top - round(1.5*labelSize.height)), cv::Point(left + round(1.5*labelSize.width), top + baseline), cv::Scalar(255, 255, 255), cv::FILLED);
              cv::putText(visImage, label, cv::Point(left, top), cv::FONT_ITALIC, 0.75, cv::Scalar(0,0,0),1);
            }
          //}
          */
          std::string windowName = "Object classification and Corner Detection and Detector Results";
          cv::namedWindow(windowName, 6);
          imshow(windowName, visImage);
          cv::waitKey(1);

          lidarThread.join();
          //cameraThread.wait();
          cameraThread.join();
          //objectDetectionThread.join();
          cameraRightIterator++;
          pclIterator++;
          std::cout << "************* End of Processing Lidar and Camera ****************" << "\n" << std::endl;
        }
      }
    }

    // Use Ego Data
    if(useEgoData)
    {

    }
  }
}