#ifndef CAMERA_H
#define CAMERA_H

#include <iostream>
#include <algorithm>
#include <opencv2/core/types.hpp>
#include <sstream>
#include <iomanip>
#include <vector>
#include <cmath>
#include <limits>
#include <thread>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

namespace CameraProcessing
{
    class Camera
    {
        public:

        // Default Constructor
        Camera() {}
        Camera(cv::Mat& image); // Default constructor
        Camera(const Camera& cameraObject); // Copy constructor
        Camera &operator=(const Camera& cameraObject); // Copy assignment constructor
        Camera (Camera &&cameraObject); // Move constructor
        Camera &operator=(Camera &&cameraObject); // Move assignment constructor
        ~Camera(); // Destructor


        //Setters & Getters
        void setImage(cv::Mat &inputImage);
        cv::Mat getImage();

        // Step 1: Input image into memory
        void inputCameraImage();

        // Step 2: detect Key Points
        void detectKeyPoints(int &detectorType, cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);

        void detectorHARRIS(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);
        void detectorSHITOMASI(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);
        void detectorFAST(cv::Mat &inputImage);
        void detectorBRISK(cv::Mat &inputImage);
        void detectorAKAZE(cv::Mat &inputImage);
        void detectorORB(cv::Mat &inputImage);
        void detectorSIFT(cv::Mat &inputImage);

        enum DETECTOR_TYPE : int
        {
            HARRIS = 1,
            SHITOMASI = 2,
            FAST = 3,
            BRISK = 4,
            AKAZE = 5,
            ORB = 6,
            SIFT = 7
        };

        // Identify Key Point Descriptors
        void descKeyPoints(cv::Mat &inputImage);

        // Match Key Points
        void matchKeyPoints(cv::Mat &currImage, cv::Mat &prevImage);

        private:
        
            cv::Mat inputImage;
    };
}



// Step 2: Convert image into a grey scale image
// Step 3: Detect key points and descriptors
// Step 4: Track Objects 
// Step 5: Estimate vehicle velocity

#endif