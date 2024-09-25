#include <string>
//#include <opencv4/opencv2/core.hpp>
#include <opencv2/core.hpp>
//#include <opencv4/opencv2/core.hpp>
#include <vector>
#include <map>
#include <pcl/io/pcd_io.h>
#include <pcl/common/common.h>

namespace Perception
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
        std::vector<cv::KeyPoint> keyPoints {}; // Key Point Vector
        std::vector<cv::KeyPoint> prevKeyPoints {}; // Prev Key Points
        cv::Mat descriptors {}; // Descriptors
        std::vector<cv::DMatch> keyPointMatches {}; // Key point matches
    };

    struct CameraImageStruct
    {
        cv::Mat imageLeft {}; // Camera Image Left (Stereo Image)
        cv::Mat imageRight {}; // Camera Image Right (Stereo Image)
        std::vector<BoundingBox> boundingBoxesLeft {}; // bounding boxes
        std::map<int, int> bbMatchesLeft {}; // bounding box matches
        std::vector<BoundingBox> boundingBoxesRight {}; // bounding boxes
        std::map<int, int> bbMatchesRight {}; // bounding box matches
    };

    // Data Structure
    struct InputStructure
    {
        pcl::PointCloud<LidarPoint> cloud {}; // Lidar Point Cloud
        std::pair<pcl::PointCloud<LidarPoint>, pcl::PointCloud<LidarPoint>> segmentedPointCloud {}; // Segmented road surface and objects
        std::vector<pcl::PointCloud<LidarPoint>> objects {}; // segmented objects

        // Stereo Camera Images
        CameraImageStruct imageStructLeft {}; // Camera Image Left
        CameraImageStruct imageStructRight {}; // Camera Image Right
    };
}