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

template<typename PointT>
class Lidar
{
    public:

        // Update the class to ensure it follows the RAII and 5-Point Rule for a solid programming standard. 
        
        // TODO: Define constructor and destructor definitions more clearly. 
        // Define default class constructor/destructor definitions
        Lidar();
        Lidar(typename pcl::PointCloud<PointT>::Ptr cloud);

        // Remove Default class declarations.
        //Lidar() = delete;

        ~Lidar() = delete;
        Lidar(const Lidar&) = delete;
        Lidar& operator=(const Lidar&) = delete;

        // Lidar Class Modifiers/Accessors
        void setPointCloud(typename pcl::PointCloud<PointT>::Ptr cloud);
        typename pcl::PointCloud<PointT>::Ptr const getPointCloud();


        // Lidar Class data manipulator funstions. Ex: Read/Write
        //typename pcl::PointCloud<PointT>::Ptr readPCLDataFile(std::string inputFile);
        typename pcl::PointCloud<PointT>::Ptr writePCLDataFile();


        typename pcl::PointCloud<PointT>::Ptr readPCLDataFile(std::string inputFile)
        {   
    
            typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

            std::fstream input(inputFile.c_str(), std::ios::in | std::ios::binary);
            if(!input.good())
            {
                std::cerr <<"Input file not loaded" << std::endl;
            }
            input.seekg(0, std::ios::beg);

            for(int i = 0; input.good() && !input.eof(); i++)
            {
                PointT point;
                input.read((char *) &point.x, 3*sizeof(float));
                input.read((char *) &point.intensity, sizeof(float));
                cloud->push_back(point);
            }

            // Use setter to set the the point cloud to this class. 
            // setPointCloud(cloud);
            std::cerr << "Loaded " << cloud->points.size () << " data points from "+inputFile << std::endl;

            return cloud;
        }

        // Point Cloud Manipulator/Processing Functions
        // Crop Pointcloud to remove outlier points.
        typename pcl::PointCloud<PointT>::Ptr cropLidarPoints(typename pcl::PointCloud<PointT>::Ptr &cloud);

        // Filter Cloud
        //typename pcl::PointCloud<PointT>::Ptr filterCloud(typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint);

typename pcl::PointCloud<PointT>::Ptr filterCloud(typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint)
{
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered (new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Filter out region that is not relevant
    typename pcl::PointCloud<PointT>::Ptr cloudRegion (new typename pcl::PointCloud<PointT>);
    
    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Vector4f(2.6, 1.7, -4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    return cloudRegion;


}
        // Step 1: Point Cloud Segmentation - RANSAC
        //std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ransacPlaneSegmentation(typename pcl::PointCloud<PointT>::Ptr &cloud);

std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> ransacPlaneSegmentation(typename pcl::PointCloud<PointT>::Ptr &cloud)
{
    std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> segmentedClouds;

    std::unordered_set<int> tempInliers;

    int numOfIterations = 250;
    float distThreshold = 0.261; 
    while(numOfIterations--)
    {
        // Create a plane using random points in the cloud
        std::unordered_set<int> inliers;

        while(inliers.size() < 3)
        {
			inliers.insert(rand()%(cloud->points.size()));
		}

        // Create a plane using 3 points from the 
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        auto itr = inliers.begin();

        x1 = cloud->points.at(*itr).x;
        y1 = cloud->points.at(*itr).y;
        z1 = cloud->points.at(*itr).z;
        
        // Increment iterator to the next point in the cloud
        itr++;
        x2 = cloud->points.at(*itr).x;
        y2 = cloud->points.at(*itr).y;
        z2 = cloud->points.at(*itr).z;

        // Increment iterator to the next point in the cloud
        itr++;
        x3 = cloud->points.at(*itr).x;
        y3 = cloud->points.at(*itr).y;
        z3 = cloud->points.at(*itr).z;

        // create the constants for the plane equation

        float A = ((y2-y1)*(z3=z1) - (z2-z1)*(y3-y1));
        float B = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1));
        float C = ((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));
        float D = -(A*x1 + B*y1 + C*z1);

        float denominator = sqrt(A*A + B*B + C*C);

        for(int i = 0; i < cloud->points.size(); i++)
        {
            float distance = fabs(A*(cloud->points.at(i).x) + B*(cloud->points.at(i).y) + C*(cloud->points.at(i).z) + D)/denominator;

            if(distance <= distThreshold)
            {
                inliers.insert(i);
            }
        }
        
        if(inliers.size() > tempInliers.size())
        {
            tempInliers = inliers;
        }
    }

    typename pcl::PointCloud<PointT>::Ptr inlierCloud (new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr outlierCloud (new typename pcl::PointCloud<PointT>);

    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(tempInliers.count(i))
        {
            inlierCloud->points.push_back(cloud->points.at(i));
        }
        else
        {
            outlierCloud->points.push_back(cloud->points.at(i));
        }
    }

    segmentedClouds.first = inlierCloud;
    segmentedClouds.second = outlierCloud;
    
    return segmentedClouds; 
}
        // Step 2: Add point cloud into a KDTree data structure to perform clustering.
        //         Insert points into KDTree
        void kdTree(); // TODO: Implement KDTree in the data structures class and use it here. 
        
        //         Clustering Algorithm.
        std::vector<typename pcl::PointCloud<PointT>::Ptr> Clustering(typename pcl::PointCloud<PointT>::Ptr &cloud, float distThreshold, int minCount, int maxCount);

        // Calibration
        //LidarCalilbration lidarCalibration;

        // Step 3 : Estimate Point Cloud Bounding box and create a bounding box around the point cloud.
        void createBoundingBox();

        // Static Variables enable the class to maintain only one copy of the variable to be used across all the instances of this class object. 
        //static int numOfIterations; // calibrate to 
        //static float distThreshold; // Calibrate to ensure correct segmentation
        // Project BB onto camera image and estimate velocity. 

        // remove Lidar points based on distance properties
        // float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane

    private:        

        //TODO: Use Heap to initialize the new cloud. 
        typename pcl::PointCloud<PointT>::Ptr pointCloud;

        //const float distThreshold = 0.261;

        //const int numOfIterations = 250;

        //static constexpr float distThreshold = 0.261; // Calibrate to ensure correct segmentation
        //int numOfIterations = 250; // calibrate to 
        //float distThreshold = 0.261; // Calibrate to ensure correct segmentation
        //int numOfIterations = 250; // calibrate to 
};
} // namespace LidarProcessing 
#endif