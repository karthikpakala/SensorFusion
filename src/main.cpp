#include "Calibration.h"
#include "Camera.h"
#include "Lidar.h"
#include "Radar.h"
#include "Tools.h"
#include <boost/filesystem/fstream.hpp>
#include <boost/filesystem/path.hpp>
#include <boost/filesystem/path_traits.hpp>
#include <chrono>
#include <filesystem>
#include <iostream>

#include <thread>
#include <mutex>
#include <future>
#include <atomic>

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
  string fileNamePre = "000000";
  string pclFileType = ".bin";
  string imageFileType = ".png";

  string fullPCLFolderPath = baseDataFolderPath + pclDataFolderPath;
  string fullImageFolderPath = baseDataFolderPath + imageDataFolderPath;

  uint16_t imageFileCount = 0;
  uint16_t pclFileCount = 0;

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

  // This Check assumes that the Camera and Lidar data was collected
  // synchronously at the same frequency and are being used accordingly. If a
  // different association technique(ex: assiciating every other camerra frame
  // with Lidar frame) is to be used, this logic needs to change.
  if (imageFileCount != pclFileCount) 
  {
    std::cerr << "Number of image files is not Equal to number of pcl files"
              << "\n"
              << "PCL File Count = " << pclFileCount << "\n"
              << "Image File Count" << imageFileCount << "\n"
              << endl;
    return 0;
  }
  // Start the Lidar and Camera Processing Loop
  else 
  {
    std::set<std::filesystem::path> sortedPCLFiles;

    // Lidar data files sort
    for (auto &file :
         std::filesystem::directory_iterator(fullPCLFolderPath))
    {
      sortedPCLFiles.insert(file.path());
    }


    std::set<filesystem::path> sortedCameraFiles;
    // Camera data files sort.
    for (auto &file : filesystem::directory_iterator(fullImageFolderPath)) 
    {
      sortedCameraFiles.insert(file.path());
    }

    // Number of CPU cores
    unsigned int nCores =  std::thread::hardware_concurrency();

    // Enable / Disable using Camera / Lidar
    bool useLidar = false;
    bool useCamera = true;

    if (useLidar)
    {
      // Load Lidatr data into buffer.
      for (auto &fileName : sortedPCLFiles) 
      {
        Lidar<pcl::PointXYZI> lidar;

        // RANSAC Segmentation parameters
        int numIterations = 50;
        float distThreshold = 0.359;

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

        /***************************************************************************************************************/
        // It is not possible to parallize this part of the code as working with point cloud requires single task to be working on it at a time to enable error free/ data race free programming.
        /*******************************************************************************************************************8*/
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


    if (useCamera) 
    {
      // Default Values
      string matcherType = "MAT_FLANN";        // MAT_BF, MAT_FLANN
      string descriptorsType = "DES_BINARY"; // DES_BINARY, DES_HOG
      string selectorType = "SEL_NN";       // SEL_NN, SEL_KNN
      int detectorType {};
      int descriptorType {};
      cout << " DETECTOR Types : 1: HARRIS | 2: SHITOMASI  | 3: FAST | 4: BRISK | 5: AKAZE | 6: ORB | 7: SIFT" << endl;
      cout << " DESCRIPTOR Types : 1: BRISK | 2: AKAZE | 3: ORB | 4: FREAK | 5: SIFT | 6: BRIEF |"<< endl;
    
      // Select Detector Type
      cout<<"Enter the Detector Type: "<< detectorType << endl;
      cin >> detectorType;

      // Select Descriptor type
      cout<<"Enter the Descriptor Type: "<< descriptorType << endl;
      cin >> descriptorType;
        
      // Current Key Points and Descriptors
      std::vector<cv::KeyPoint> keyPoints{};
      cv::Mat descriptors {};

      // Previous Key Points and Descriptors
      std:vector<cv::KeyPoint> prevKeyPoints {};
      cv::Mat prevDesc {};

      // Matches from prev and current frames
      std::vector<cv::DMatch> matches {};

      // Load input image into the buffer.
      for (auto &fileName : sortedCameraFiles) 
      {
        cv::Mat inputImage = cv::imread(fileName.c_str());
        CameraProcessing::Camera cameraObject;


        if(imageBuffer.size() == imageBufferSize)
        {
          imageBuffer.erase(imageBuffer.begin());
        }
        imageBuffer.push_back(inputImage);

        
        cameraObject.detectKeyPoints(detectorType, inputImage, keyPoints);

        
        cameraObject.descriptorKeyPoints(inputImage, keyPoints, descriptorType, descriptors);

        cameraObject.matchKeyPoints(keyPoints, prevKeyPoints, descriptors, prevDesc, matches,
                                              descriptorsType, matcherType, selectorType);

        std::cout << "Key Point Match count = " << matches.size() << "\n" << std::endl;
        cv::Mat visImage = inputImage.clone();
        cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
        std::string windowName = "Corner Detection and Detector Results";
        cv::namedWindow(windowName, 6);
        imshow(windowName, visImage);
        cv::waitKey(10);
        std::cout << "Key PointSize = " <<  keyPoints.size() << std::endl;

        // Capture previous key points and descriptors
        cout << "Prev Key Points Count" << 
        prevKeyPoints = keyPoints;
        prevDesc = descriptors;
        cout << "Camera image name = " << fileName.c_str() << endl;
      }
    }
  }
}