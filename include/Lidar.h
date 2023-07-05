#ifndef LIDAR_H
#define LIDAR_H

#include <iostream>
#include <algorithm>
#include <string>
#include <fstream>
#include <cstdlib>
#include <unordered_set>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "data_structure.h"
//#include "Calibration.h"
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/crop_box.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/common/transforms.h>
#include <pcl/impl/point_types.hpp>

#include "eigen3/Eigen/Dense"

using Eigen::Vector4f;

namespace LidarProcessing
{
class Lidar
{
    public:
        
        // TODO: Define constructor and destructor definitions more clearly. 
        // Define default class constructor/destructor definitions
        Lidar();
        Lidar(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);

        // Remove Default class declarations.
        //Lidar() = delete;
        ~Lidar() = delete;
        Lidar(const Lidar&) = delete;
        Lidar& operator=(const Lidar&) = delete;

        // Lidar Class Modifiers/Accessors
        void setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud);
        pcl::PointCloud<pcl::PointXYZI>::Ptr const getPointCloud();


        // Lidar Class data manipulator funstions. Ex: Read/Write
        pcl::PointCloud<pcl::PointXYZI>::Ptr readPCLDataFile(std::string inputFile);
        pcl::PointCloud<pcl::PointXYZI>::Ptr writePCLDataFile();


        // Point Cloud Manipulator/Processing Functions
        // Crop Pointcloud to remove outlier points.
        pcl::PointCloud<pcl::PointXYZI>::Ptr cropLidarPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);

        // Filter Cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint);

        // Step 1: Point Cloud Segmentation - RANSAC
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> ransacPlaneSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int numOfIterations, float distanceThreshold);

        // Step 2: Add point cloud into a KDTree data structure to perform clustering.
        //         Insert points into KDTree
        void kdTree(); // TODO: Implement KDTree in the data structures class and use it here. 
        
        //         Clustering Algorithm.
        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float distThreshold, int minCount, int maxCount);

        // Calibration
        //LidarCalilbration lidarCalibration;

        // Step 3 : Estimate Point Cloud Bounding box and create a bounding box around the point cloud.
        void createBoundingBox();

        // Project BB onto camera image and estimate velocity. 

        // remove Lidar points based on distance properties
        // float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane

    private:        

        //TODO: Use Heap to initialize the new cloud. 
        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
};
} // namespace LidarProcessing 
#endif