#include "Calibration.h"
#include "Camera.h"
#include "Lidar.h"
#include "Radar.h"
#include "Tools.h"

#include <chrono>
#include <filesystem>
#include <iostream>
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
using namespace LidarProcessing;

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
    string baseDataFolderPath = "/home/karthikpakala/Pers-Projects/Data/Kitti-data1"; // File path for linux
  #else
    string baseDataFolderPath = "/Users/karthikpakala/Projects/Data/KITTI-data3"; // File path for macosx
  #endif


  string pclDataFolderPath = "/velodyne_points/data/";
  string imageDataFolderPath = "/image_02/data/";
  string egoFolderPath = "/oxts/data/";
  string fileNamePre = "000000";
  string pclFileType = ".bin";
  string imageFileType = ".png";
  string egoFileType = ".txt";
  


  string fullPCLFolderPath = baseDataFolderPath + pclDataFolderPath;
  string fullImageFolderPath = baseDataFolderPath + imageDataFolderPath;
  string fullEgoFolderPath = baseDataFolderPath + egoFolderPath;

  uint16_t imageFileCount = 0;
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
  for (auto &file : std::filesystem::directory_iterator(fullImageFolderPath)) 
  {
    ++imageFileCount;
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
  if (imageFileCount != pclFileCount || egoFileCount != imageFileCount || egoFileCount != pclFileCount) 
  {
    std::cerr << "Number of image files is not Equal to number of pcl files"
              << "\n"
              << "PCL File Count = " << pclFileCount << "\n"
              << "Image File Count" << imageFileCount << "\n"
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
    std::set<filesystem::path> sortedCameraFiles;
    for (auto &file : filesystem::directory_iterator(fullImageFolderPath)) 
    {
      sortedCameraFiles.insert(file.path());
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

    // RANSAC Segmentation parameters
    int numIterations = 50;
    float distThreshold = 0.359;

    if(useLidar && useCamera)
    {
      fileCount = pclFileCount;
      auto cameraIterator = sortedCameraFiles.begin();
      auto pclIterator = sortedPCLFiles.begin();

      uint16_t cameraCount = 0;
      while(cameraIterator != sortedCameraFiles.end() && pclIterator != sortedPCLFiles.end())
      {
        std::vector<std::future<void>> futures;
        Lidar<pcl::PointXYZI> lidarObject;
        CameraProcessing::Camera cameraObject;

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
        while (!viewer->wasStopped()) 
        {
          std::cout << "*************** Start of Lidar and Camera Processing **************" << std::endl;
          CameraAngle cameraAngle = XY;
          tools->initCamera(cameraAngle, viewer);
          viewer->removeAllPointClouds();
          viewer->removeAllShapes();

          //std::future<void> lidarThread = std::async(std::launch::deferred, &Lidar<pcl::PointXYZI>::readPCLDataFile, lidarObject, (*pclIterator).string(), std::ref(viewer));
          std::thread lidarThread = std::thread(&Lidar<pcl::PointXYZI>::readPCLDataFile, &lidarObject, (*pclIterator).string(), std::ref(viewer));
          cv::Mat inputImage = cv::imread((*cameraIterator).string());

          cameraCount++;

          //std::future<void> cameraThread = std::async(std::launch::deferred, &CameraProcessing::Camera::cameraProcessing, cameraObject, std::ref(inputImage), std::ref(detectorType), 
          //                                            std::ref(descriptorType), std::ref(selectorType), std::ref(matcherType), std::ref(keyPoints), std::ref(descriptors), 
          //                                            std::ref(prevKeyPoints), std::ref(prevDescriptors), std::ref(matches), std::ref(matchDescriptorsType), 
          //                                            std::ref(cameraCount));

          std::promise<std::vector<cv::KeyPoint>> prevKeyPointsPromise;
          std::promise<cv::Mat> prevDescriptorsPromise;
          std::promise<std::vector<cv::DMatch>> matchesPromise;

          std::future<std::vector<cv::KeyPoint>> prevKeyPointsFuture = prevKeyPointsPromise.get_future();
          std::future<cv::Mat> prevDescriptorsFuture = prevDescriptorsPromise.get_future();
          std::future<std::vector<cv::DMatch>> matchesFuture = matchesPromise.get_future();

          // std::cout << "//***********************//" << std::endl;
          // std::cout << "Key Points before function call = " << keyPoints.size() << std::endl;
          // std::cout << "Prev Key Points before function call = " << prevKeyPoints.size() << std::endl;
          // std::cout << "//***********************//" << std::endl;
          std::thread cameraThread = std::thread(&CameraProcessing::Camera::cameraProcessing, cameraObject, 
                                                  std::ref(inputImage), 
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
                                                  std::ref(cameraCount));
          // Update previous Key Points and Descriptors - Get data from other thread to use it in next iteration - Pending
          // In Development /////
          prevKeyPoints = prevKeyPointsFuture.get();
          prevDescriptors = prevDescriptorsFuture.get();
          std::vector<cv::DMatch> testMatches = matchesFuture.get();
          // In Development /////

          // std::cout << "//***********************//" << std::endl;          //std::cout << "Key Point Size = " <<  keyPoints.size() << std::endl;
          // std::cout << "Previous Key Point Size = " <<  prevKeyPoints.size() << std::endl;
          // std::cout << "Key Point Match count = " << matches.size() << std::endl;
          // std::cout << "Matches Count from Camera Thread = " << testMatches.size() << std::endl;
          // std::cout << "//***********************//" << std::endl;    

          cv::Mat visImage = inputImage.clone();
          cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);

          std::string windowName = "Corner Detection and Detector Results";
          cv::namedWindow(windowName, 6);
          imshow(windowName, visImage);
          cv::waitKey(10);

          //lidarThread.wait();
          lidarThread.join();
          //cameraThread.wait();
          cameraThread.join();
          cameraIterator++;
          pclIterator++;
          std::cout << "************* End of Processing Lidar and Camera ****************" << "\n" << std::endl;
        }
      }
    }
    /*
    else if(useLidar && !useCamera)
    {
      // Load Lidatr data into buffer.
      for (auto &fileName : sortedPCLFiles) 
      {
        Lidar<pcl::PointXYZI> lidar;



        // TODO: Check if these new variables can be overloaded to inorporate
        // new features into the new keyword to enable static loading of the
        // memory without having to re-create a new memory location at each
        // call.
        //  move semantics to be used to udpate this.

        Tools *tools;
        pcl::visualization::PCLVisualizer::Ptr viewer(
            new pcl::visualization::PCLVisualizer("3D Viewer"));
        // viewer->getRenderWindow()->GlobalWarningDisplayOff(); // suppress VTK
        // warnings

        auto fileIterator = sortedPCLFiles.begin();

        // PCL Visualization
        while (!viewer->wasStopped()) 
        {
          CameraAngle cameraAngle = XY;
          tools->initCamera(cameraAngle, viewer);
          viewer->removeAllPointClouds();
          viewer->removeAllShapes();

          std::vector<std::future<void>> futures;

        **************************************************************************************************************
        // It is not possible to parallize this part of the code as working with point cloud requires single task to be working 
        // on it at a time to enable error free/ data race free programming.
        *******************************************************************************************************************8
        // Use move semantics to enable 

        std::future<void> future = std::async(std::launch::deferred, &Lidar<pcl::PointXYZI>::readPCLDataFile, lidar, (*fileIterator).string(), std::ref(viewer));

        future.wait();

          fileIterator++;
          if (fileIterator == sortedPCLFiles.end()) 
          {
            return 0;
          }
          viewer->spinOnce();
        }
      }
    }
    */
    /*
    else if (!useLidar && useCamera) 
    {
      // Default Values
      string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
      string matchDescriptorsType = "DES_BINARY"; // DES_BINARY, DES_HOG
      string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
      int detectorType {};
      int descriptorType {};

      CameraProcessing::Camera cameraObject;

      // Initialize Camera detection parameters
      cameraObject.init(matcherType, descriptorsType, selectorType);
        
      // Current Key Points and Descriptors
      std::vector<cv::KeyPoint> keyPoints{};
      cv::Mat descriptors {};

      // Previous Key Points and Descriptors
      std::vector<cv::KeyPoint> prevKeyPoints {};
      cv::Mat prevDescriptors {};

      // Matches from prev and current frames
      std::vector<cv::DMatch> matches {};

      uint8_t count = 0;
      // Load input image into the buffer.
      for (auto &fileName : sortedCameraFiles) 
      {
        count++;
        cv::Mat inputImage = cv::imread(fileName.c_str());
        cameraObject.cameraProcessing(inputImage, detectorType, descriptorsType, selectorType, matcherType, keyPoints, descriptors, prevKeyPoints,prevDescriptors, matches, matchDescriptorsType, cameraCount);

        //if(imageBuffer.size() == imageBufferSize)
        //{
        //  imageBuffer.erase(imageBuffer.begin());
        //}
        //imageBuffer.push_back(inputImage);

        std::cout << "Key Point Match count = " << matches.size() << "\n" << std::endl;
        cv::Mat visImage = inputImage.clone();
        cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Corner Detection and Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(10);
        std::cout << "Key PointSize = " <<  keyPoints.size() << std::endl;

        // Capture previous key points and descriptors
        prevKeyPoints = keyPoints;
        prevDesc = descriptors;

        cout << "Prev Key Points Count = " << prevKeyPoints.size() << endl;
        cout << "Camera image name = " << fileName.c_str() << endl;
      }
    }
    */

    if(useEgoData)
    {
        float_t  lat;
        float_t  lon;
        float_t  alt;
        float_t  roll;
        float_t  pitch;
        float_t  yaw;
        float_t  vel_north;
        float_t  vel_east;
        float_t  vel_forward;
        float_t  vel_left;
        float_t  vel_up;
        float_t  ax;
        float_t  ay;
        float_t  az;
        float_t  a_forward;
        float_t  a_left;
        float_t  a_upward;
        float_t  ang_rate_x;
        float_t  ang_rate_y;
        float_t  ang_rate_z;
        float_t  ang_rate_forward;
        float_t  ang_rate_left;
        float_t  ang_rate_upward;
        float_t  pos_accuracy;
        float_t  vel_accuracy;
        int32_t     navstat;
        int32_t     numstats;
        int32_t     posmode;
        int32_t     velmode;
        int32_t     orimode;
    }
  }
}