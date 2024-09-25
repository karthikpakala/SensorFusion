#include "Camera.h"


using namespace std;
using namespace Perception::CameraProcessing;
Perception::CameraProcessing::Camera::Camera(cv::Mat &image)
{
    inputImage = image;
}

Perception::CameraProcessing::Camera::Camera(const Camera &cameraObject)
{
    inputImage = std::move(cameraObject.inputImage);
}

Perception::CameraProcessing::Camera &Perception::CameraProcessing::Camera::operator=(const Camera &cameraObject)
{
    if (&cameraObject == this)
    {
        return *this;
    }
    inputImage = std::move(cameraObject.inputImage);

    return *this;
}

Perception::CameraProcessing::Camera::Camera(Camera &&cameraObject)
{
    inputImage = std::move(cameraObject.inputImage);
}

Perception::CameraProcessing::Camera &Camera::operator=(Camera &&cameraObject)
{
    if (this == &cameraObject)
    {
        return *this;
    }

    inputImage = std::move(cameraObject.inputImage);

    return *this;
}

Perception::CameraProcessing::Camera::~Camera()
{
}

void Perception::CameraProcessing::Camera::init(int &detectorType, int &descriptorType)
{

          
    std::cout << "HOG Detectors : HARRIS | Shi-Tomasi | SIFT | SURF" << std::endl;
    std::cout << "Binary Detectors : FAST | BRIEF | ORB | BRISK | FREAK" << std::endl;
    std::cout << "HOG Descriptors : SIFT | SURF | HARRIS " << std::endl;
    std::cout << "Binary Descriptors : BRISK | BRIEF | ORB | FREAK | AKAZE" << std::endl;
    std::cout << "\n" << std::endl;

    std::cout << "Fastest combinations of Detector/Descriptors are: " << std::endl;
    std::cout << "FAST/BRISK | FAST/BRIEF | FAST/ORB" << std::endl;
    std::cout << "\n" << std::endl;

    cout << "Matcher is set to use FLANN based matcher - Only SURF and SIFT are allowed for now - Additional additions are set to be added soon" << endl;
    cout << " DETECTOR Types : 1: HARRIS | 2: SHITOMASI  | 3: FAST | 4: BRISK | 5: AKAZE | 6: ORB | 7: SIFT" << endl;
    cout << " DESCRIPTOR Types : 1: SIFT | 2: AKAZE | 3: ORB | 4: FREAK | 5: BRISK | 6: BRIEF |"<< endl;
    std::cout << "\n" << std::endl;

    // Select Detector Type
    cout<<"Enter the Detector Type: "<< detectorType << endl;
    cin >> detectorType;

    // Select Descriptor type
    cout<<"Enter the Descriptor Type: "<< descriptorType << endl;
    cin >> descriptorType;
}

void Perception::CameraProcessing::Camera::cameraProcessing(cv::Mat &inputImage, 
                                                int &detectorType, 
                                                int &descriptorType, 
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
                                                uint16_t &count,
                                                std::string &modelWeightsPath,
                                                std::string &modelClassesPath,
                                                std::string &modelConfigurationPath)
{
    cameraDataLock.lock();
    // Detect Key Points
    std::cout << "Image dimensions = " << " | " << "Rows = " << inputImage.rows << " | " << "Cols = " << inputImage.cols << std::endl;
    // Create ROI for the image.
    int roiRows = inputImage.rows;
    int roiCols = inputImage.cols;

    int roiStartRow = 2 * roiRows/5;
    int roiStartCol = 0;

    int numOfRows = 3 * roiRows/5;
    int numOfCols = roiCols;
    
    int roiEndRow = roiRows;
    int roiEndCol = roiCols;

    int width = roiCols;
    int roiHeight = 2 * (roiRows/3);

    cv::Point roiTopLeft (roiStartRow, roiStartCol);
    cv::Point roiBottomRight (roiEndRow, roiEndCol);

    regionOfInterest = cv::Rect(roiStartRow, roiStartCol, numOfRows, numOfCols);
    cv::rectangle(inputImage,regionOfInterest,cv::Scalar(255,0,0),1,8,0);
    //regionOfInterest = cv::Rect(inputImage, roiTopLeft, roiBottomRight, cv::Scalar(0, 255, 0), cv::LINE_8, 0);

    detectKeyPoints(detectorType, inputImage, keyPoints);
    
    // Descriptors for Key Points
    descriptorKeyPoints(inputImage, keyPoints, descriptorType, descriptors);
    
    // Match Descriptors if count > 1
    if(count > 1)
    {
        std::cout << "Descriptors size = " << descriptors.size() << " | " << " Prev Descriptors size = " << prevDescriptors.size() << std::endl;
        descriptors.convertTo(descriptors, CV_32F); // Necessary to convert Binary descriptors to float to be used with FLANN
        prevDescriptors.convertTo(prevDescriptors, CV_32F); // Necessary to convert Binary descriptors to float to be used with FLANN
        matchKeyPoints(keyPoints, prevKeyPoints, descriptors, prevDescriptors, matches,
                                              matchDescriptorsType, matcherType, selectorType);
    }
    
    //detectObjects(inputImage, modelWeightsPath, modelClassesPath, modelConfigurationPath);

    prevKeyPointsPromise.set_value(keyPoints);
    prevDescriptorsPromise.set_value(descriptors);
    matchesPromise.set_value(matches);

    //prevKeyPoints = keyPoints;
    //prevDescriptors = descriptors;

    matches.clear();
    cameraDataLock.unlock();
}

void Perception::CameraProcessing::Camera::detectKeyPoints(int &detectorType, cv::Mat &image, std::vector<cv::KeyPoint> &keyPoints)
{
    switch (detectorType)
    {
    case HARRIS:
        detectorHARRIS(image, keyPoints);
        break;
    case SHITOMASI:
        detectorSHITOMASI(image, keyPoints);
        break;
    case FAST:
        detectorFAST(image, keyPoints);
        break;
    case BRISK:
        detectorBRISK(image, keyPoints);
        break;
    case AKAZE:
        detectorAKAZE(image, keyPoints);
        break;
    case ORB:
        detectorORB(image, keyPoints);
        break;
    case SIFT:
        detectorSIFT(image, keyPoints);
    case GPU_FAST:
        detectorGPUFAST(image, keyPoints);
        break;

    default:
        std::cout << "Invalid Detector Selected" << std::endl;
    }
}

void Perception::CameraProcessing::Camera::detectorGPUFAST(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Mat greyImage, img_cv16fc3, img_cv16fc3_gpu{};
    cv::cvtColor(inputImage, greyImage, cv::COLOR_RGB2GRAY);
    inputImage.convertTo(img_cv16fc3, CV_16FC3);
    cv::cuda::GpuMat gpuMat{}, gpuMat_cv32fc3{}, gpuMat_cv16fc3;
    gpuMat.upload(img_cv16fc3);
    gpuMat.convertTo(gpuMat_cv32fc3, CV_32FC3);
    cv::cuda::convertFp16(gpuMat_cv32fc3, gpuMat_cv16fc3);
    gpuMat_cv16fc3.download(img_cv16fc3_gpu);


    cv::Ptr<cv::cuda::FastFeatureDetector> detector = cv::cuda::FastFeatureDetector::create(10, true);
    double time = (double)cv::getTickCount();
    detector->detect(img_cv16fc3, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "GPU FAST Detector Extraction time : " << 1000 * time / 1.0 << " ms " << std::endl;
}

void Perception::CameraProcessing::Camera::descriptorKeyPoints(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints, int &descType, cv::Mat &descriptors)
{
    cv::Ptr<cv::DescriptorExtractor> extractor;
    if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::BRISK_DESC)
    {
        int threshold = 30;        // FAST AKAZE detection threshold Scale
        int octaves = 3;           // Detection octatves (use 0 to do single scale)
        float patternScale = 1.0f; // Aply this scale to the pattern used for sampling the neighbors
        extractor = cv::BRISK::create(threshold, octaves, patternScale);
        std::cout << "BRISK Descriptor Selected" << std::endl;
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
        std::cout << "AKAZE Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::ORB_DESC)
    {
        extractor = cv::ORB::create(500, 1.2, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
        std::cout << "ORB Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::FREAK_DESC)
    {
        extractor = cv::xfeatures2d::FREAK::create(true, true, 22.0F, 4);
        std::cout << "FREAK Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::SIFT_DESC)
    {
        extractor = cv::SiftDescriptorExtractor::create(0, 3, 0.04, 10.0, 1.6);
        std::cout << "SIFT Descriptor Selected" << std::endl;
    }
    else if (descType == CameraProcessing::Camera::DESCRIPTOR_TYPE::BRIEF_DESC)
    {
        extractor = cv::xfeatures2d::BriefDescriptorExtractor::create();
        std::cout << "BRIEF Descriptor Selected" << std::endl;
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

void Perception::CameraProcessing::Camera::detectorHARRIS(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
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
}

void Perception::CameraProcessing::Camera::detectorSHITOMASI(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
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
}

void Perception::CameraProcessing::Camera::detectorFAST(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Mat greyImage;
    cv::cvtColor(inputImage, greyImage, cv::COLOR_RGB2GRAY);

    cv::Ptr<cv::FastFeatureDetector> detector = cv::FastFeatureDetector::create(10, true);
    double time = (double)cv::getTickCount();
    detector->detect(greyImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "FAST Detector Extraction time : " << 1000 * time / 1.0 << " ms " << std::endl;
}

void Perception::CameraProcessing::Camera::detectorBRISK(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::BRISK> detector = cv::BRISK::create(10, true);
    double time = (double)cv::getTickCount();
    detector->detect(inputImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "BRISK key point detection extraction in " << 1000 * time / 1.0 << "ms" << std::endl;
}

void Perception::CameraProcessing::Camera::detectorAKAZE(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::AKAZE> detector = cv::AKAZE::create();
    double time = (double)cv::getTickCount();
    detector->detect(inputImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "AKAZE Detector Extraction" << 1000 * time / 1.0 << "ms" << std::endl;
}

void Perception::CameraProcessing::Camera::detectorORB(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::ORB> detector = cv::ORB::create();
    double time = (double)cv::getTickCount();
    detector->detect(inputImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "ORB Detection extraction time = " << 1000 * time / 1.0 << "ms" << std::endl;
}

void Perception::CameraProcessing::Camera::detectorSIFT(cv::Mat &inputImage, std::vector<cv::KeyPoint> &keyPoints)
{
    cv::Ptr<cv::SiftFeatureDetector> detector = cv::SiftFeatureDetector::create();
    double time = (double)cv::getTickCount();
    detector->detect(inputImage, keyPoints);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "SIFT feature detection time = " << 1000 * time / 1.0 << "ms" << std::endl;
}

void Perception::CameraProcessing::Camera::matchKeyPoints(std::vector<cv::KeyPoint> &keyPoints, std::vector<cv::KeyPoint> &prevKeyPoints, cv::Mat &descriptors, cv::Mat &prevDescriptors, std::vector<cv::DMatch> &matches,
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
        
        int counter = 0;
        for (const auto& it : knnMatch)
        {
            counter++;
            if(it[0].distance < distRatio * it[1].distance)
            {
                matches.push_back(it[0]);
            }
        }
    }
    std::cout << "Matches Count matcher function= " << matches.size() << std::endl;
}

void Perception::CameraProcessing::Camera::detectObjects(cv::Mat &inputImage, std::string &modelWeightsPath, std::string &modelClassesPath, std::string modelConfigurationPath, 
                                                        std::promise<vector<Perception::DataStructure::BoundingBox>> &bBoxesPromise, std::promise<vector<string>> &classesPromise, 
                                                        std::promise<vector<int>> &classIdsPromise, std::promise<vector<float>> &confidencesPromise, std::promise<vector<cv::Rect>> &bondingBoxesPromise)
{
    std::vector<Perception::DataStructure::BoundingBox> bBoxes {};
    float nmsThreshold = 0.8;

    // Step 1: Retrieve and load neural network
    vector<string> classes{};
    ifstream ifs(modelClassesPath.c_str());
    std::string line;
    while(getline(ifs, line))
    {
        classes.push_back(line);
    }

    // Step 2: load neural network
    cv::dnn::Net net = cv::dnn::readNetFromDarknet(modelConfigurationPath, modelWeightsPath);
    net.setPreferableBackend(cv::dnn::DNN_BACKEND_CUDA); //CPU: DNN_BACKEND_OPENCV | GPU: DNN_BACKEND_CUDA
    net.setPreferableTarget(cv::dnn::DNN_TARGET_CUDA); //DNN_TARGET_CPU | GPU: DNN_TARGET_CUDA | DNN_TARGET_CUDA_FP16

    // Step 3:Create Blob out of input image to pass into YOLO model. 
    cv::Mat blob;
    double scaleFactor = 1/255.0;
    cv::Size size = cv::Size(416, 416);
    cv::Scalar mean = cv::Scalar(0,0,0);
    bool swapRB = false;
    bool crop = false;
    cv::dnn::blobFromImage(inputImage, blob, scaleFactor, size, mean, swapRB, crop);

    // Step 4: Run Forward Pass through YOLO Network
    vector<cv::String> modelOutputNames{};
    
    // get indices of output layers i.e., layers with unconnected outputs
    vector<int> modelOutputLayers = net.getUnconnectedOutLayers();
    
    // get names of all layers in the network
    vector<cv::String> layerNames = net.getLayerNames(); 

    modelOutputNames.resize(modelOutputLayers.size());
    for(size_t i = 0; i < modelOutputLayers.size(); ++i)
    {
        modelOutputNames[i] = layerNames[modelOutputLayers[i] - 1];
    }

    double time = (double)cv::getTickCount();
    // invoke forward propagation through network
    vector<cv::Mat> netOutput{};
    net.setInput(blob);
    net.forward(netOutput, modelOutputNames);
    time = ((double)cv::getTickCount() - time) / cv::getTickFrequency();
    std::cout << "Object Detection time = " << 1000 * time / 1.0 << "ms" << std::endl;
    // scan through all bounding boxes and keep only the ones with high condfidence
    float confidenceThreshold = 0.90;
    vector<int> classIds{};
    vector<float> confidences{};
    vector<cv::Rect> boundingBoxes {};

    for(size_t i = 0; i < netOutput.size(); ++i)
    {
        float *data = (float*)netOutput[i].data;
        for(int j = 0; j < netOutput[i].rows; ++j, data += netOutput[i].cols)
        {
            cv::Mat scores = netOutput[i].row(j).colRange(5, netOutput[i].cols);
            cv::Point classId;
            double confidence;

            // Get the value and location of the maximum score
            cv::minMaxLoc(scores, 0, &confidence, 0, &classId);
            if(confidence > confidenceThreshold)
            {
                cv::Rect box;
                int cx, cy;
                cx = (int)(data[0] * inputImage.cols);
                cy = (int)(data[0] * inputImage.rows);
                box.width = (int)(data[2] * inputImage.cols);
                box.height = (int)(data[3] * inputImage.rows);
                box.x = cx - box.width/2; // left
                box.y = cy - box.height/2; // top

                boundingBoxes.push_back(box);
                classIds.push_back(classId.x);
                confidences.push_back((float)confidence);
            }
        }
    }

    // Perform non-maxima suppression
    vector<int> indices {};
    cv::dnn::NMSBoxes(boundingBoxes, confidences, confidenceThreshold, nmsThreshold, indices);

    for(auto it = indices.begin(); it != indices.end(); ++it)
    {
        Perception::DataStructure::BoundingBox bBox;
        bBox.roi = boundingBoxes[*it];
        bBox.classID = classIds[*it];
        bBox.confidence = confidences[*it];
        bBox.boxID = (int)bBoxes.size(); // Zero based unique id for this BBox

        bBoxes.push_back(bBox);
    }
    
    bBoxesPromise.set_value(bBoxes);
    classesPromise.set_value(classes);
    classIdsPromise.set_value(classIds);
    confidencesPromise.set_value(confidences);
    bondingBoxesPromise.set_value(boundingBoxes);
}