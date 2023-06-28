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
class Lidar
{
    

    public:

        //void readPCLDataFile(pcl::PointCloud<LidarPoint> &cloud, std::string inputFile);
        // Read files in correct order
        pcl::PointCloud<pcl::PointXYZI>::Ptr readPCLDataFile(std::string inputFile);


        // Filter point cloud to remove outliers
        pcl::PointCloud<pcl::PointXYZI>::Ptr cropLidarPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud);


        // Filter Cloud
        pcl::PointCloud<pcl::PointXYZI>::Ptr filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint);

        // Lidar Functions
        // Steps:
        // Step 1: Point Cloud Segmentation - RANSAC
        std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> ransacPlaneSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int numOfIterations, float distanceThreshold);

        std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float distThreshold, int minCount, int maxCount);


        // Step 2: Point Cloud Clustering - 
        //         Insert points into KDTree
        void kdTree();

        // Calibration
        //LidarCalilbration lidarCalibration;

        // Bounding Box determination
        void createBoundingBox();

        // Project BB onto camera image and estimate velocity. 

        // remove Lidar points based on distance properties
        // float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane

    private:

        pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
        //pcl::PointCloud<LidarPoint> cloud;
    // Private functions
};
#endif