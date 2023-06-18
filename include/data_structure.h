#include <iostream>
#include <string>
#include <opencv4/opencv2/core.hpp>
#include <vector>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>
#include <fstream>

struct LidarPoint
{
    // x = x coordinate // y = y coordinate // z = z coordinate // i = intensity
    double x_coordinate, y_coordinate, z_coordinate, intensity;
    // index
    int index;
};

struct BoundingBox
{
    // Bounding Box ID
    int boxID;

    // Track ID
    int trackID;

    // Region of Interest
    cv::Rect roi;

    // Class ID
    int classID;

    // Confidence
    double confidence;

    // Lidar Point Vector
    pcl::PointCloud<LidarPoint> lidarPoints;

    // Key Point Vector
    std::vector<cv::KeyPoint> keyPoints;

    // Key Point Matches
    std::vector<cv::DMatch> keyPointMatches;
};

struct DataStruct
{
    pcl::PointCloud<LidarPoint> lidarPoints;

    std::vector<BoundingBox> boundingBoxes;

    std::vector<cv::KeyPoint> keyPoints;

    std::vector<cv::DMatch> keyPointMatches;

};