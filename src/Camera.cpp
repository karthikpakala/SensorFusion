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

CameraProcessing::Camera::Camera(const Camera &cameraObject)
{
    inputImage = std::move(cameraObject.inputImage);
}

CameraProcessing::Camera &Camera::operator=(const Camera &cameraObject)
{
    if (&cameraObject == this)
    {
        return *this;
    }
    inputImage = std::move(cameraObject.inputImage);

    return *this;
}

CameraProcessing::Camera::Camera(Camera &&cameraObject)
{
    inputImage = std::move(cameraObject.inputImage);
}

CameraProcessing::Camera &Camera::operator=(Camera &&cameraObject)
{
    if (this == &cameraObject)
    {
        return *this;
    }

    inputImage = std::move(cameraObject.inputImage);

    return *this;
}

CameraProcessing::Camera::~Camera()
{
}

void CameraProcessing::Camera::init(int &detectorType, int &descriptorType)
{
      cout << "Matcher is set to use FLANN based matcher - Only SURF and SIFT are allowed for now - Additional additions are set to be added soon" << endl;
      cout << " DETECTOR Types : 1: HARRIS | 2: SHITOMASI  | 3: FAST | 4: BRISK | 5: AKAZE | 6: ORB | 7: SIFT" << endl;
      cout << " DESCRIPTOR Types : 1: BRISK | 2: AKAZE | 3: ORB | 4: FREAK | 5: SIFT | 6: BRIEF |"<< endl;
    
      // Select Detector Type
      cout<<"Enter the Detector Type: "<< detectorType << endl;
      cin >> detectorType;

      // Select Descriptor type
      cout<<"Enter the Descriptor Type: "<< descriptorType << endl;
      cin >> descriptorType;
}

void CameraProcessing::Camera::cameraProcessing(cv::Mat &inputImage, int &detectorType, int &descriptorType, string &selectorType, string &matcherType, vector<cv::KeyPoint> &keyPoints, cv::Mat &descriptors, vector<cv::KeyPoint> &prevKeyPoints, cv::Mat &prevDescriptors,  std::vector<cv::DMatch> &matches, string &matchDescriptorsType, uint16_t &count)
{
    // Detect Key Points
    detectKeyPoints(detectorType, inputImage, keyPoints);
    
    // Descriptors for Key Points
    descriptorKeyPoints(inputImage, keyPoints, descriptorType, descriptors);

    // Match Descriptors if count > 1
    if(count > 1)
    {
      matchKeyPoints(keyPoints, prevKeyPoints, descriptors, prevDescriptors, matches,
                                              matchDescriptorsType, matcherType, selectorType);
    }
    std::
}
void CameraProcessing::Camera::detectKeyPoints(int &detectorType, cv::Mat &image, std::vector<cv::KeyPoint> &keyPoints)
{
    switch (detectorType)
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
        detectorFAST(image, keyPoints);
        break;
    case BRISK:
        std::cout << "BRISK" << std::endl;
        detectorBRISK(image, keyPoints);
        break;
    case AKAZE:
        std::cout << "AKAZE" << std::endl;
        detectorAKAZE(image, keyPoints);
        break;
    case ORB:
        std::cout << "ORB" << std::endl;
        detectorORB(image, keyPoints);
        break;
    case SIFT:
        std::cout << "SIFT" << std::endl;
        detectorSIFT(image, keyPoints);
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
    for (int j = 0; j < dst_norm.rows; j++)
    {
        for (int i = 0; i < dst_norm.cols; i++)
        {

            int response = (int)dst_norm.at<float>(j, i);
            if (response > minResponse)
            {
                cv::KeyPoint newKeyPoint;
                newKeyPoint.pt = cv::Point2f(i, j);
                newKeyPoint.size = 2 * sobel;
                newKeyPoint.response = response;

                bool boolOverlap = false;

                auto startTime = std::chrono::steady_clock::now();
                for (int index = 0; index < keyPoints.size(); index++)
                {

                    double kptOverlap = cv::KeyPoint::overlap(newKeyPoint, keyPoints.at(index));

                    // double kptOverlap = 0.00345;
                    // std::cout << " Over lap = " << kptOverlap << std::endl;
                    if (kptOverlap > maxOverLap)
                    {
                        boolOverlap = true;
                        if (newKeyPoint.response > keyPoints.at(index).response)
                        {
                            keyPoints.at(index) = newKeyPoint;
                            break;
                        }
                    }
                }
                auto endTime = std::chrono::steady_clock::now();

                auto elapsedTime = std::chrono::duration_cast<std::chrono::microseconds>(endTime - startTime);
                std::cout << "camera processing time = " << elapsedTime.count() << std::endl;
                if (!boolOverlap)
                {
                    keyPoints.push_back(newKeyPoint);
                }
            }
        }
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << "Harris Key Point extraction in " << 1000 * t / 1.0 << "milliseconds" << std::endl;
    std::cout << "Harris Key Point Count = " << keyPoints.size() << std::endl;
    windowName = "Harris corner Detection Results";
    cv::namedWindow(windowName);

    // cv::Mat visImage = dst_norm_scaled.clone();
    // cv::drawKeypoints(greyImage, keyPoints, visImage);
    // cv::imshow(windowName, visImage);
    // cv::waitKey(10);
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

    cv::Mat greyImage;
    cvtColor(inputImage, greyImage, cv::COLOR_RGB2GRAY);
    cv::goodFeaturesToTrack(greyImage, corners, maxCorners, qualityLevel, minDistance, cv::Mat(), blockSize, false, k);

    for (auto it = corners.begin(); it != corners.end(); it++)
    {
        cv::KeyPoint newKeyPoint;
        newKeyPoint.pt = cv::Point2f((*it).x, (*it).y);
        newKeyPoint.size = blockSize;
        keyPoints.push_back(newKeyPoint);
    }

    t = ((double)cv::getTickCount() - t) / cv::getTickFrequency();
    std::cout << " Shi-To-Masi detection with n =" << keyPoints.size() << "key points in " << 1000 * t / 1.0 << "ms" << std::endl;

    // cv::Mat visImage = inputImage.clone();
    // cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // std::string windowName = "Shi-To Masi Corner Detector Results";
    // cv::namedWindow(windowName, 6);
    // imshow(windowName, visImage);
    // cv::waitKey(0);
}

void CameraProcessing::Camera::detectorFAST(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Mat greyImage;
    cv::cvtColor(inputImage, greyImage, cv::COLOR_RGB2GRAY);

    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(10, true);
    double time = (double)cv::getTickCount();
    detector->detect(greyImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "FAST Detectorextraction time" << 1000 * time / 1.0 << " ms " << std::endl;
    // std::string windowName = "FAST FEatue Detection";
    // cv::namedWindow(windowName, 5);
    // cv::Mat visImage = inputImage.clone();
    // cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imshow(windowName, visImage);
    // cv::waitKey(10);
}

void CameraProcessing::Camera::detectorBRISK(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::BRISK> detector = cv::BRISK::create(10, true);
    double time = (double)cv::getTickCount();
    detector->detect(inputImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "BRISK key point detection extraction in " << 1000 * time / 1.0 << "ms" << std::endl;
    // std::string windowName = "BRISK Detection Results" ;
    // cv::namedWindow(windowName, 5);
    // cv::Mat visImage = inputImage.clone();
    // cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imshow(windowName, visImage);
    // cv::waitKey(10);
}

void CameraProcessing::Camera::detectorAKAZE(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
    double time = (double)cv::getTickCount();
    detector->detect(inputImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "AKAZE Detector Extraction" << 1000 * time / 1.0 << "ms" << std::endl;
    // std::string windowName = "AKAZE Detection Results" ;
    // cv::namedWindow(windowName, 5);
    // cv::Mat visImage = inputImage.clone();
    // cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imshow(windowName, visImage);
    // cv::waitKey(10);
}

void CameraProcessing::Camera::detectorORB(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    double time = (double)cv::getTickCount();
    detector->detect(inputImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "ORB Detection extraction time = " << 1000 * time / 1.0 << "ms" << std::endl;
    // std::string windowName = "ORB Detection";
    // cv::namedWindow(windowName, 5);
    // cv::Mat visImage = inputImage.clone();
    // cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imshow(windowName, visImage);
    // cv::waitKey(10);
}

void CameraProcessing::Camera::detectorSIFT(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::SiftFeatureDetector> detector = cv::SiftFeatureDetector::create();
    double time = (double)cv::getTickCount();
    detector->detect(inputImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "SIFT feature detection time = " << 1000 * time / 1.0 << "ms" << std::endl;
    // std::string windowName = "SIFT Detection Results";
    // cv::Mat visImage = inputImage.clone();
    // cv::drawKeypoints(inputImage, keyPoints, visImage, cv::Scalar::all(-1), cv::DrawMatchesFlags::DRAW_RICH_KEYPOINTS);
    // cv::imshow(windowName, visImage);
    // cv::waitKey(10);
}

void CameraProcessing::Camera::descriptorKeyPoints(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints, int &descType, cv::Mat &descriptors)
{
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::BRISK_DESC)
    {
        int threshold = 30;        // FAST AKAZE detection threshold Scale
        int octaves = 3;           // Detection octatves (use 0 to do single scale)
        float patternScale = 1.0f; // Aply this scale to the pattern used for sampling the neighbors
        extractor = cv::BRISK::create(threshold, octaves, patternScale);
        std::cout << " BRISK Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::AKAZE_DESC)
    {
        int descriptorSize = 0;
        int descriptorChannels = 0;
        float threshold = 0.001f;
        int nOctaves = 4;
        int nOctaveLayers = 4;

        extractor = cv::AKAZE::create(cv::AKAZE::DESCRIPTOR_MLDB, descriptorSize, descriptorChannels,
                                      threshold, nOctaves, nOctaveLayers, cv::KAZE::DIFF_PM_G2);
        std::cout << " AKAZE Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::ORB_DESC)
    {
        extractor = cv::ORB::create(500, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
        std::cout << " ORB Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::FREAK_DESC)
    {
        extractor = cv::xfeatures2d::FREAK::create(true, true, 22.0F, 4);
        std::cout << " FREAK Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::SIFT_DESC)
    {
        extractor = cv::SiftDescriptorExtractor::create(0, 3, 0.04, 10.0, 1.6);
        std::cout << " SIFT Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::BRIEF_DESC)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
        std::cout << " BRIEF Descriptor Selected" << std::endl;
    }
    else
    {
        std::cout << "Unexpected Descriptor Selected" << std::endl;
    }

    double time = (double)cv::getTickCount();
    extractor->compute(inputImage, keyPoints, descriptors);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "Descriptor Extraction in " << 1000 * time / 1.0 << "ms" << std::endl;
}

void CameraProcessing::Camera::matchKeyPoints(std::vector<cv::KeyPoint> &keyPoints, std::vector<cv::KeyPoint> &prevKeyPoints, cv::Mat &descriptors, cv::Mat &prevDescriptors, std::vector<cv::DMatch> &matches,
                                              std::string matchDescriptorsType, std::string matcherType, std::string selectorType)
{
    // configure matcher
    bool crossCheck = true;
    cv::Ptr<cv::DescriptorMatcher> matcher;

    if (matcherType.compare("MAT_BF") == 0)
    {
        int normType = cv::NORM_HAMMING;
        matcher = cv::BFMatcher::create(normType, crossCheck);
    }
    else if (matcherType.compare("MAT_FLANN") == 0)
    {
        matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::FLANNBASED);
    }

        // perform matching task
    if (selectorType.compare("SEL_NN") == 0)
    { // nearest neighbor (best match)

        matcher->match(descriptors, prevDescriptors, matches); // Finds the best match for each descriptor in desc1
    }
    else if (selectorType.compare("SEL_KNN") == 0)
    { // k nearest neighbors (k=2)
        int k = 2;
        double distRatio = 0.8;
        std::vector<std::vector<cv::DMatch>> knnMatch ;
        matcher->knnMatch(descriptors, prevDescriptors, knnMatch, k);
        
        std::cout << "KNN Matches Count = " << knnMatch.size() << std::endl;

        for (const auto& it : knnMatch)
        {
            if(it[0].distance < distRatio * it[1].distance)
            {
                matches.push_back(it[0]);
            }
        }
    }
}