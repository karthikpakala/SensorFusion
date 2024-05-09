#ifndef CALIBRATION_H
#define CALIBRATION_H

#include <iostream>
#include <cstdio>
#include <string>
#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
//#include <opencv2/xfeatures2d.hpp>
//#include <opencv2/xfeatures2d/nonfree.hpp>


class Calibration
{
    public:

    // object detection
    std::string dataPath = "..KITTI-data/"; // Update with correct path once available.
    std::string yoloBasePath = dataPath + "dat/yolo/";
    std::string yoloClassesFile = yoloBasePath + "coco.names";
    std::string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    std::string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // calibration data for camera and lidar
    cv::Mat P_rect_00 {3,4,cv::DataType<double>::type}; // 3x4 projection matrix after rectification
    cv::Mat R_rect_00 {4,4,cv::DataType<double>::type}; // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT {4,4,cv::DataType<double>::type}; // rotation matrix and translation vector

    void initializeMatrices();
};

#endif