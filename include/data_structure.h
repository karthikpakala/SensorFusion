// #pragma once

#ifndef DATA_STRUCTURE_H
#define DATA_STRUCTURE_H

#include <string>
#include <vector>
#include <map>

#include <opencv2/core.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

namespace Perception
{
    namespace DataStructure
    {
        // OXTS Data
        struct OxtsData
        {
            float_t  lat = 0.0;
            float_t  lon = 0.0;
            float_t  alt = 0.0;
            float_t  roll = 0.0;
            float_t  pitch = 0.0;
            float_t  yaw = 0.0;
            float_t  vel_north = 0.0;
            float_t  vel_east = 0.0;
            float_t  vel_forward = 0.0;
            float_t  vel_left = 0.0;
            float_t  vel_up = 0.0;
            float_t  ax = 0.0;
            float_t  ay = 0.0;
            float_t  az = 0.0;
            float_t  a_forward = 0.0;
            float_t  a_left = 0.0;
            float_t  a_upward = 0.0;
            float_t  ang_rate_x = 0.0;
            float_t  ang_rate_y = 0.0;
            float_t  ang_rate_z = 0.0;
            float_t  ang_rate_forward = 0.0;
            float_t  ang_rate_left = 0.0;
            float_t  ang_rate_upward = 0.0;
            float_t  pos_accuracy = 0.0;
            float_t  vel_accuracy = 0.0;
            int32_t  navStat = 0;
            int32_t  numStats = 0;
            int32_t  posMode = 0;
            int32_t  velMode = 0;
            int32_t  oriMode = 0.0;
        };

        // Lidar Point
        struct LidarPoint
        {
            // x = x coordinate // y = y coordinate // z = z coordinate // i = intensity
            double x_coordinate, y_coordinate, z_coordinate, intensity;
            // index
            int index;
        };

        // Bounding Box
        struct BoundingBox
        {
            int boxID = 0; // Bounding Box ID
            int trackID = 0; // Track ID
            cv::Rect roi {}; // Region of Interest
            int classID = 0; // Class ID 
            double confidence = 0.0; // Confidence
            std::vector<cv::KeyPoint> keyPoints {}; // Bounding Box Key Point Vector
            std::vector<cv::KeyPoint> prevKeyPoints {}; // Bounding Box Prev Key Points
            cv::Mat descriptors {}; // Descriptors
            std::vector<cv::DMatch> keyPointMatches {}; // Bounding Box Key point matches
        };

        // Camera Image Structure
        struct CameraImageStruct
        {
            cv::Mat image {}; // Camera Image Left (Stereo Image)
            std::vector<cv::KeyPoint> keyPoints {}; // Image KeyPoints
            std::vector<cv::KeyPoint> prevKeyPoints{}; // Prev Image Keypoints
            cv::Mat descriptors {}; // Descriptors
            cv::Mat prevDescriptors {}; // Previous Descriptors
            std::vector<cv::DMatch> keyPointMatches {}; // Bounding Box Key point matches
            std::vector<BoundingBox> boundingBoxes {}; // bounding boxes
            std::map<int, int> bbMatches {}; // bounding box matches
        };

        struct egoDataStructure
        {
            OxtsData oxtsData{}; // Ego Vehicle Data - IMU
        };

        // Input Data Structure
        struct InputStructure
        {
            pcl::PointCloud<LidarPoint> cloud {}; // Lidar Point Cloud
            std::pair<pcl::PointCloud<LidarPoint>, pcl::PointCloud<LidarPoint>> segmentedPointCloud {}; // Segmented road surface and objects
            std::vector<pcl::PointCloud<LidarPoint>> segmentedObjects {}; // segmented objects

            // Stereo Camera Images
            CameraImageStruct imageStructLeft {}; // Camera Image Left
            CameraImageStruct imageStructRight {}; // Camera Image Right
        };
    } // DataStructure
} // Perception

#endif // DATA_STRUCTURE_H