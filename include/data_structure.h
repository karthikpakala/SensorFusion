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

namespace DataStructure
{
    struct LidarPoint
    {
        // x = x coordinate // y = y coordinate // z = z coordinate // i = intensity
        double x_coordinate, y_coordinate, z_coordinate, intensity;
        // index
        int index;
    };

    struct BoundingBox
    {
        int boxID; // Bounding Box ID
        int trackID; // Track ID
        cv::Rect roi; // Region of Interest
        int classID; // Class ID 
        double confidence; // Confidence
        pcl::PointCloud<LidarPoint> cloud; // PointCloud
        std::vector<cv::KeyPoint> keyPoints; // Key Point Vector
        std::vector<cv::DMatch> keyPointMatches; // Key Point Matches
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
}