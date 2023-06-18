#ifndef LIDAR_H
#define LIDAR_H

#include <iostream>
#include <algorithm>
#include <string>

#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include "data_structure.h"
//#include "Calibration.h"
//#include <pcl/filters/extract_indices.h>
//#include <pcl/filters/voxel_grid.h>
//#include <pcl/filters/crop_box.h>
//#include <pcl/kdtree/kdtree.h>
//#include <pcl/segmentation/sac_segmentation.h>
//#include <pcl/segmentation/extract_clusters.h>
//#include <pcl/common/transforms.h>
using namespace pcl;
using namespace std;

class Lidar
{

    public:

        void readPCLDataFile(vector<LidarPoint> &lidarPoints, std::string inputFile);

        vector<LidarPoint> cropLidarPoints(vector<LidarPoint> &lidarPoints);

        // Process Lidar Input
        void processLidarInput();

        // Lidar Functions
        // Steps:
        // Step 1: Point Cloud Segmentation - RANSAC
        void ransac();

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

        //vector<LidarPoint> lidarPoints;
        pcl::PointCloud<LidarPoint> lidarPoints;

    // Private functions
};
#endif