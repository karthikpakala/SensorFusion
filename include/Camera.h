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
#include <future>

#include <opencv2/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/xfeatures2d.hpp>
#include <opencv2/xfeatures2d/nonfree.hpp>

using namespace std;
namespace CameraProcessing
{
    class Camera
    {
        public:
        
        std::mutex cameraDataLock;
        // Default Constructor
        Camera() {}
        Camera(cv::Mat& image); // Default constructor
        Camera(const Camera& cameraObject); // Copy constructor
        Camera &operator=(const Camera& cameraObject); // Copy assignment constructor
        Camera (Camera &&cameraObject); // Move constructor
        Camera &operator=(Camera &&cameraObject); // Move assignment constructor
        ~Camera(); // Destructor

        void init(int &detectorType, int &descriptorType);
        void cameraProcessing(cv::Mat &inputImage, 
                                int &detectorType, 
                                int &descriptorsType, 
                                string &selectorType, 
                                string &matcherType, 
                                vector<cv::KeyPoint> &keyPoints, 
                                cv::Mat &descriptors, 
                                vector<cv::KeyPoint> &prevKeyPoints, 
                                std::promise<std::vector<cv::KeyPoint>> &&prevKeyPointsPromise, 
                                cv::Mat &prevDescriptors, 
                                std::promise<cv::Mat> &&prevDescriptorsPromise, 
                                std::vector<cv::DMatch> &matches, 
                                std::promise<std::vector<cv::DMatch>> &&matchesPromise, 
                                string &matchDescriptorsType,
                                uint16_t &count);
        //Setters & Getters
        void setImage(cv::Mat &inputImage);
        cv::Mat getImage();

        // Step 1: Input image into memory
        void inputCameraImage();

        // Step 2: detect Key Points
        void detectKeyPoints(int &detectorType, cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);

        void detectorHARRIS(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);
        void detectorSHITOMASI(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);
        void detectorFAST(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);
        void detectorBRISK(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);
        void detectorAKAZE(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);
        void detectorORB(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);
        void detectorSIFT(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints);

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

        enum DESCRIPTOR_TYPE : int
        {   
            SIFT_DESC = 1,
            AKAZE_DESC = 2,
            ORB_DESC = 3,
            FREAK_DESC = 4,
            BRISK_DESC = 5,
            BRIEF_DESC = 6
        };

        // Identify Key Point Descriptors
        void descriptorKeyPoints(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints, int &descType, cv::Mat &descriptors);

        // Match Key Points
        //void matchKeyPoints(cv::Mat &currImage, cv::Mat &prevImage);
        void matchKeyPoints(std::vector<cv::KeyPoint> &keyPointsSource, std::vector<cv::KeyPoint> &keyPointRef, cv::Mat &descSource, cv::Mat &descRef, std::vector<cv::DMatch> &matches,
                                              std::string descType, std::string matcherType, std::string selectorType);
        private:
        
            cv::Mat inputImage;
    };
}



// Step 2: Convert image into a grey scale image
// Step 3: Detect key points and descriptors
// Step 4: Track Objects 
// Step 5: Estimate vehicle velocity

#endif