#ifndef LIDAR_H
#define LIDAR_H

#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <iostream>
#include <string>
#include <unordered_set>
#include <mutex>
#include <thread>
//#include <pcl-1.13/pcl/common/common.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/crop_box.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl-1.13/pcl/gpu/segmentation>
//#include <pcl-1.13/pcl/gpu/containers>
//#include <pcl-1.13/pcl/gpu/containers/device_memory.h>

// #include <pcl/impl/point_types.hpp>
// #include <pcl/kdtree/kdtree_flann.h>
// #include <pcl/common/transforms.h>

#if __linux__
//#include <cuda.h>
#endif


#include "Tools.h"

#include "eigen3/Eigen/Dense"

using Eigen::Vector4f;
// using namespace Tooling;
namespace Perception
{
namespace LidarProcessing
{

template <typename PointT>
  class Lidar
  {

  private:
    typename pcl::PointCloud<PointT>::Ptr pointCloud {};

    int numberOfIterations = 0;
    float distanceThreshold = 0.0;

    std::mutex lidarDataLock;

  public:

  // Class constructor/destructor 
    Lidar();
    Lidar(typename pcl::PointCloud<PointT>::Ptr &inputCloud);
    Lidar(const Lidar &lidarObject);
    Lidar &operator=(const Lidar &lidarObject);
    Lidar(Lidar &&lidarObject) noexcept; // No except as per CPP guidelines
    Lidar &operator=(Lidar &&lidarObject) noexcept; // No except as per CPP guidelines.
    ~Lidar();


    // member functions
    void setPointCloud(typename pcl::PointCloud<PointT>::Ptr &inputCloud);
    typename pcl::PointCloud<PointT>::Ptr getPointCloud();
    //typename pcl::PointCloud<PointT>::Ptr readPCLDataFile(std::string inputFile, pcl::visualization::PCLVisualizer::Ptr &viewer);
    void readPCLDataFile(std::string inputFile, pcl::visualization::PCLVisualizer::Ptr &viewer);
    typename pcl::PointCloud<PointT>::Ptr writePCLDataFile();
    void processPointCloud(typename pcl::PointCloud<PointT>::Ptr &inputCloud, pcl::visualization::PCLVisualizer::Ptr &viewer);
    void setNumberOfIterations(int &numOfIterations);
    int getNumberOfIterations();
    void setDistanceThreshold(float &distanceThreshold);
    float getDistanceThreshold();
    typename pcl::PointCloud<PointT>::Ptr cropLidarPoints(typename pcl::PointCloud<PointT>::Ptr &cloud);
    typename pcl::PointCloud<PointT>::Ptr filterCloud(typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint);
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ransacPlaneSegmentation(typename pcl::PointCloud<PointT>::Ptr &cloud, pcl::visualization::PCLVisualizer::Ptr &viewer);
    //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ransacPCLLibrary(typename pclPointCloud<PointT>::Ptr &cloud, pcl::visualization::PCLVisualizer::Ptr &viewer);
    std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr &cloud, float distThreshold, int minCount, int maxCount, pcl::visualization::PCLVisualizer::Ptr &viewer);

    // Step 2: Add point cloud into a KDTree data structure to perform clustering.
    //         Insert points into KDTree
    void kdTree(); // TODO: Implement KDTree in the data structures class and use
                   // it here.
    // Clustering Algorithm.
    // Step 3 : Estimate Point Cloud Bounding box and create a bounding box around
    // the point cloud.
    void createBoundingBox();
  };

  // template class Lidar<pcl::PointXYZI>;

} // namespace LidarProcessing
} // namespace Perception
#endif