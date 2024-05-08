#include <iostream>
#include <string>
//#include <opencv4/opencv2/core.hpp>
#include <opencv2/core.hpp>
//#include <opencv4/opencv2/core.hpp>
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
    pcl::PointCloud<LidarPoint> cloud;

    // Key Point Vector
    std::vector<cv::KeyPoint> keyPoints;

    // Key Point Matches
    std::vector<cv::DMatch> keyPointMatches;
};

struct DataStruct
{
    //1. Lidar Cloud
    //2. bounding boxes
    //3. ROI
    //4. camera image
    //5. keypoints
    //6. matches
    //7. descriptors
    //8. bounding boxes
    //9. BB class ID
    //10. bounding boxes matches

    cv::Mat image; // Camera Image

    pcl::PointCloud<LidarPoint> cloud; // Associated Lidar Points

    std::vector<cv::KeyPoint> keyPoints; //  Key Points

    std::vector<cv::DMatch> keyPointMatches; // Key point matches
    cv::Mat descriptors; // Key point descriptors

    std::map<int, int> bbMatches; // bounding box matches
    std::vector<BoundingBox> boundingBoxes; // bounding boxes
};