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
    string baseDataFolderPath = "/home/karthik/Projects/data/KITTI-data3"; // File path for linux
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
    bool useLidar = true;
    bool useCamera = false;


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

         // auto startTime = std::chrono::steady_clock::now();

          // run on a single thread
          //std::thread pclThread(&Lidar<pcl::PointXYZI>::readPCLDataFile, lidar, (*fileIterator).string(), std::ref(viewer));
          //pclThread.join();

          // default
          //lidar.readPCLDataFile((*fileIterator).string(), viewer);
          
          // Multiple threading start
          //
          /*
          std::vector<std::thread> threads;

          for(int i = 0; i < nCores; i++)
          {
            threads.emplace_back(std::thread(&Lidar<pcl::PointXYZI>::readPCLDataFile, lidar, (*fileIterator).string(), std::ref(viewer)));
          }

          for(auto &t : threads)
          {
            t.join();
          }
          // Multiple threads end
          */
         
          std::vector<std::future<void>> futures;

          /*
          for(int i = 0; i < nCores; i++)
          {
            futures.emplace_back(
                  std::async(std::launch::deferred, &Lidar<pcl::PointXYZI>::readPCLDataFile, lidar, (*fileIterator).string(), std::ref(viewer)));
          }

          for(const std::future<void> &ftr : futures)
          {
            ftr.wait();
          }
          */

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

        //  auto endTime = std::chrono::steady_clock::now();
        //  auto elapsedTime =
        //      std::chrono::duration_cast<std::chrono::milliseconds>(endTime -
        //                                                            startTime);
          //cout << " Time Eleapsed for Lidar Processing = "
          //     << elapsedTime.count() << " milliseconds " << endl;
        }
      }
    }


    if (useCamera) 
    {
      //std::mutex _mutex;
      //_mutex.lock();

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


        std::vector<cv::KeyPoint> keyPoints{};
        cameraObject.detectorHARRIS(imageBuffer.back(), keyPoints);
        //cameraObject.detectorSHITOMASI(imageBuffer.back(), keyPoints);

        std::cout << "Key PointSize = " <<  keyPoints.size() << std::endl;

        cout << "Camera image name = " << fileName.c_str() << endl;
        //cv::imshow("input Image", inputImage);
        //cv::waitKey(10);
      }
      //_mutex.unlock();
    }
  }
}