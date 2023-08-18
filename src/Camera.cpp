#include "Camera.h"
#include <opencv2/core.hpp>
#include <opencv2/core/base.hpp>
#include <opencv2/core/hal/interface.h>
#include <opencv2/core/matx.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/features2d.hpp>
#include <opencv2/highgui.hpp>
#include <numeric>
#include <opencv2/imgproc.hpp>
#include <chrono>

using namespace CameraProcessing;

CameraProcessing::Camera::Camera(cv::Mat &image)
{
    inputImage = image;
}

CameraProcessing::Camera::Camera(const Camera& cameraObject)
{
    inputImage = std::move(cameraObject.inputImage);   
}

CameraProcessing::Camera &Camera::operator=(const Camera& cameraObject)
{
    if(&cameraObject == this)
    {
        return *this;
    }
    inputImage = std::move(cameraObject.inputImage);

    return *this;
}

CameraProcessing::Camera::Camera(Camera&& cameraObject)
{
    inputImage = std::move(cameraObject.inputImage);
}

CameraProcessing::Camera &Camera::operator=(Camera&& cameraObject)
{
    if(this == &cameraObject)
    {
        return *this;
    }

    inputImage = std::move(cameraObject.inputImage);
    
    return *this;
}

CameraProcessing::Camera::~Camera()
{

}

void CameraProcessing::Camera::detectKeyPoints(int &detectorType, cv::Mat &image, std::vector<cv::KeyPoint> &keyPoints)
{
    switch(detectorType)
    {
        case HARRIS:
            std::cout << "HARRIS Detector" << std::endl;
            detectorHARRIS(image, keyPoints);
            break;
        case SHITOMASI:
            std::cout << "SHITOHMASI Detector" << std::endl;
            detectorSHITOMASI(image, keyPoints);
            break;
        case FAST:
            std::cout << "FAST" << std::endl;
            detectorFAST(image);
            break;
        case BRISK:
            std::cout << "BRISK" << std::endl;
            detectorBRISK(image);
            break;
        case AKAZE:
            std::cout << "AKAZE" << std::endl;
            detectorAKAZE(image);
            break;
        case ORB:
            std::cout << "ORB" << std::endl;
            detectorORB(image);
            break;
        case SIFT:
            std::cout << "SIFT" << std::endl;
            detectorSIFT(image);
            break;

        default:
            std::cout << "Invalid Detector Selected" << std::endl;
    }
}

void CameraProcessing::Camera::detectorHARRIS(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Mat dst, dst_norm, dst_norm_scaled;

    int minResponse = 120;
    dst = cv::Mat::zeros(inputImage.size(), CV_32FC1);

    int blockSize = 2;
    int sobel = 3;
    int alpha = 0;
    int beta = 255;


    cv::Mat greyImage;
        
    cv::cvtColor(inputImage, greyImage, cv::COLOR_RGB2GRAY);

    double detectorParameter = 0.04;
    cv::cornerHarris(greyImage, dst, blockSize, sobel, detectorParameter, cv::BORDER_DEFAULT);
    cv::normalize(dst, dst_norm, alpha, beta, cv::NORM_MINMAX, CV_32FC1, cv::Mat());
    cv::convertScaleAbs(dst_norm, dst_norm_scaled);

    std::string windowName = "HARRIS Corner Detection Response Matrix";
    cv::namedWindow(windowName, 4);
    cv::imshow(windowName, dst_norm_scaled);
    cv::waitKey(10);

    double maxOverLap = 0.0;
    double t = (double)cv::getTickCount();
    for(int j = 0; j < dst_norm.rows;j++)
    {
        for(int i = 0; i < dst_norm.cols;i++)
        {

            int response = (int)dst_norm.at<float>(j,i);
            if(response > minResponse)
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i,j);
                newKeyPoint.size = 2*sobel;
                newKeyPoint.response = response;

                bool boolOverlap = false;

                auto startTime = std::chrono::steady_clock::now();
                for(int index = 0; index < keyPoints.size(); index++)
                {
                    
                   double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, keyPoints.at(index));

                   // double kptOverlap = 0.00345;
                    //std::cout << " Over lap = " << kptOverlap << std::endl;
                    if(kptOverlap > maxOverLap)
                    {
                        boolOverlap = true;
                        if(newKeyPoint.response > keyPoints.at(index).response)
                        {
                            keyPoints.at(index) = newKeyPoint;
                            break;
                        }
                    }
                }
                auto endTime = std::chrono::steady_clock::now();

                auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
                std::cout << "camera processing time = " << elapsedTime.count() << std::endl;
                if(!boolOverlap)
                {
                    keyPoints.push_back(newKeyPoint);
                }
            }
        }
    }

    t = ((double)cv::getTickCount() - t)/cv::getTickFrequency();
    std::cout << "Harris Key Point extraction in " << 1000*t/1.0 << "milliseconds" << std::endl;
    std::cout << "Harris Key Point Count = " << keyPoints.size() << std::endl;
    windowName = "Harris corner Detection Results";
    cv::namedWindow(windowName);
    
    cv::Mat visImage = dst_norm_scaled.clone();
    cv::drawKeypoints(greyImage, keyPoints, visImage);
    cv::imshow(windowName, visImage);
    cv::waitKey(10);
}

void CameraProcessing::Camera::detectorSHITOMASI(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    // compute detector parameters based on image size
    int blockSize = 4;
    double maxOverlap = 0.0; // max opssible overlap between two featuresin %
    double minDistance = (1.0 - maxOverlap) * blockSize;
    int maxCorners = inputImage.rows * inputImage.cols / std::max(1.0, minDistance);

    double qualityLevel = 0.01; // minimal accepted quality of image corners
    double k = 0.04;

    // apply corner detection
    double t = (double)cv::getTickCount();
    std::vector<cv::Point2f> corners;
    cv::goodFeaturesToTrack(inputImage, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    for(auto it = corners.begin(); it != corners.end(); it++)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keyPoints.push_back(newKeyPoint);
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << " Shi-To-Masi detection with n =" << keyPoints.size() << "key points in " << 1000 * t/ 1.0 << "ms" << std::endl;

    cv::Mat visImage = inputImage.clone();
    cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    std::string windowName = "Shi-To Masi Corner Detector Results";
    cv::namedWindow(windowName, 6);
    imshow(windowName, visImage);
    cv::waitKey(0);
}

void CameraProcessing::Camera::detectorFAST(cv::Mat &inputImage)
{

}

void CameraProcessing::Camera::detectorBRISK(cv::Mat &inputImage)
{

}

void CameraProcessing::Camera::detectorAKAZE(cv::Mat &inputImage)
{

}

void CameraProcessing::Camera::detectorORB(cv::Mat &inputImage)
{

}

void CameraProcessing::Camera::detectorSIFT(cv::Mat & inputImage)
{

}