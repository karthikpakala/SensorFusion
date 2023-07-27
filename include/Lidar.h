#ifndef LIDAR_H
#define LIDAR_H

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>

#include "data_structure.h"
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
//#include "Calibration.h"
#include <pcl/common/transforms.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/impl/point_types.hpp>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include "Tools.h"

#include "eigen3/Eigen/Dense"

using Eigen::Vector4f;
// using namespace Tooling;
namespace LidarProcessing 
{

template <typename PointT> 
class Lidar 
{

  private:
    // TODO: Use Heap to initialize the new cloud.
    typename pcl::PointCloud<PointT>::Ptr pointCloud{};

  public:
    Lidar();

    int numberOfIterations = 250;
    float distanceThreshold = 0.261;
  // Default Constructor  
  
    Lidar(typename pcl::PointCloud<PointT>::Ptr &inputCloud, const int &numOfIterations, const float &distThreshold); 


  *****************************************************************************
  //  Applying Rule of 5 to define the Copy/Move/Copy/Move Assignment/
  //  Destructor//
  // Methods
  //  Definition : If one of the methods is being defined, the other two have //
  //  to be defined to ensure the memory-management strategy is being followed
  //  // correctly. //
  *****************************************************************************

  // Copy Constructor
  Lidar(const Lidar &lidarObject); 

  // Copy Assignment constructor
  Lidar &operator=(const Lidar &lidarObject); 

  // Move constructor
  Lidar(Lidar &&lidarObject) noexcept; // No except as per CPP guidelines

  // Move assignment constructor
  Lidar &operator=(Lidar &&lidarObject) noexcept; // No except as per CPP guidelines.

  // Destructor
  ~Lidar();



  // Lidar Class Modifiers/Accessors
  typename pcl::PointCloud<PointT>::Ptr writePCLDataFile();

  typename pcl::PointCloud<PointT>::Ptr readPCLDataFile(std::string inputFile);

  // Point Cloud Manipulator/Processing Functions
  // Crop Pointcloud to remove outlier points.
  typename pcl::PointCloud<PointT>::Ptr cropLidarPoints(typename pcl::PointCloud<PointT>::Ptr &cloud);

  // Filter Cloud
  typename pcl::PointCloud<PointT>::Ptr filterCloud(typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint);

  // Step 1: Point Cloud Segmentation - RANSAC
  std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ransacPlaneSegmentation(typename pcl::PointCloud<PointT>::Ptr &cloud);

  // Step 2: Add point cloud into a KDTree data structure to perform clustering.
  //         Insert points into KDTree
  void kdTree(); // TODO: Implement KDTree in the data structures class and use
                 // it here.

  //         Clustering Algorithm.
  std::vector<typename pcl::PointCloud<PointT>::Ptr>
  Clustering(typename pcl::PointCloud<PointT>::Ptr &cloud, float distThreshold, int minCount, int maxCount);

  // Step 3 : Estimate Point Cloud Bounding box and create a bounding box around
  // the point cloud.
  void createBoundingBox();
};
} // namespace LidarProcessing
#endif