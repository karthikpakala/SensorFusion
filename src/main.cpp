#include <iostream>
#include <filesystem>
#include "Lidar.h"
#include "Radar.h"
#include "Camera.h"

using namespace std;


int main(int argv, char **argc)
{

    // object detection
    //string yoloBasePath = dataPath + "dat/yolo/";
    //string yoloClassesFile = yoloBasePath + "coco.names";
    //string yoloModelConfiguration = yoloBasePath + "yolov3.cfg";
    //string yoloModelWeights = yoloBasePath + "yolov3.weights";

    // calibration data for camera and lidar
    cv::Mat P_rect_00(3,4,cv::DataType<double>::type); // 3x4 projection matrix after rectification
    cv::Mat R_rect_00(4,4,cv::DataType<double>::type); // 3x3 rectifying rotation to make image planes co-planar
    cv::Mat RT(4,4,cv::DataType<double>::type); // rotation matrix and translation vector
    
    RT.at<double>(0,0) = 7.533745e-03; RT.at<double>(0,1) = -9.999714e-01; RT.at<double>(0,2) = -6.166020e-04; RT.at<double>(0,3) = -4.069766e-03;
    RT.at<double>(1,0) = 1.480249e-02; RT.at<double>(1,1) = 7.280733e-04; RT.at<double>(1,2) = -9.998902e-01; RT.at<double>(1,3) = -7.631618e-02;
    RT.at<double>(2,0) = 9.998621e-01; RT.at<double>(2,1) = 7.523790e-03; RT.at<double>(2,2) = 1.480755e-02; RT.at<double>(2,3) = -2.717806e-01;
    RT.at<double>(3,0) = 0.0; RT.at<double>(3,1) = 0.0; RT.at<double>(3,2) = 0.0; RT.at<double>(3,3) = 1.0;
    
    R_rect_00.at<double>(0,0) = 9.999239e-01; R_rect_00.at<double>(0,1) = 9.837760e-03; R_rect_00.at<double>(0,2) = -7.445048e-03; R_rect_00.at<double>(0,3) = 0.0;
    R_rect_00.at<double>(1,0) = -9.869795e-03; R_rect_00.at<double>(1,1) = 9.999421e-01; R_rect_00.at<double>(1,2) = -4.278459e-03; R_rect_00.at<double>(1,3) = 0.0;
    R_rect_00.at<double>(2,0) = 7.402527e-03; R_rect_00.at<double>(2,1) = 4.351614e-03; R_rect_00.at<double>(2,2) = 9.999631e-01; R_rect_00.at<double>(2,3) = 0.0;
    R_rect_00.at<double>(3,0) = 0; R_rect_00.at<double>(3,1) = 0; R_rect_00.at<double>(3,2) = 0; R_rect_00.at<double>(3,3) = 1;
    
    P_rect_00.at<double>(0,0) = 7.215377e+02; P_rect_00.at<double>(0,1) = 0.000000e+00; P_rect_00.at<double>(0,2) = 6.095593e+02; P_rect_00.at<double>(0,3) = 0.000000e+00;
    P_rect_00.at<double>(1,0) = 0.000000e+00; P_rect_00.at<double>(1,1) = 7.215377e+02; P_rect_00.at<double>(1,2) = 1.728540e+02; P_rect_00.at<double>(1,3) = 0.000000e+00;
    P_rect_00.at<double>(2,0) = 0.000000e+00; P_rect_00.at<double>(2,1) = 0.000000e+00; P_rect_00.at<double>(2,2) = 1.000000e+00; P_rect_00.at<double>(2,3) = 0.000000e+00;    

    // misc
    //double sensorFrameRate = 10.0 / imgStepWidth; // frames per second for Lidar and camera
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    //vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time
    bool bVis = false;            // visualize results


    // Data file path definitions.
    string baseDataFolderPath = "../KITTI-data";
    string pclDataFolderPath = "/velodyne_points/data/";
    string imageDataFolderPath = "/image_02/data/";
    string fileNamePre = "000000";
    string pclFileType = ".bin";
    string imageFileType = ".png";

    string fullPCLFolderPath = baseDataFolderPath + pclDataFolderPath;
    string fullImageFolderPath = baseDataFolderPath + imageDataFolderPath;

    uint16_t imageFileCount = 0;
    uint16_t pclFileCount = 0;

    // Initiate Lidar points vector
    vector<LidarPoint> lidarPoints;

    // Initiate ac image
    cv::Mat inputImage;

    // PCL File counter
    for(auto& file : filesystem::directory_iterator(fullPCLFolderPath))
    {
        ++pclFileCount;
    }
    
    // Image File Counter
    for(auto& file : filesystem::directory_iterator(fullImageFolderPath))
    {
        ++imageFileCount;
    }

    // This Check assumes that the Camera and Lidar data was collected synchronously at the same frequency and are being used accordingly. 
    // If a different association technique(ex: assiciating every other camerra frame with Lidar frame) is to be used, this logic needs to change.
    if(!(imageFileCount == pclFileCount))
    {
        std::cerr << "Number of image files is not Equal to number of pcl files" << "\n" << "PCL File Count = " << pclFileCount << "\n" << "Image File Count" << imageFileCount << "\n" << endl;
        return 0;
    }
    else
    {     
        // Load Lidatr data into buffer. 
        for(auto& file : filesystem::directory_iterator(fullPCLFolderPath))
        {            
            Lidar lidar;
            lidar.readPCLDataFile(lidarPoints, file.path());
            cout << "Lidar PCD size = " << lidarPoints.size() << endl;

            // Write code to process lidar points
            //lidar.cropLidarPoints(lidarPoints);
            cout << "Lidar Points after cropping = " << lidarPoints.size() << endl;
        }

        // Load input image into the buffer. 
        for(auto& file : filesystem::directory_iterator(fullImageFolderPath))
        {
            inputImage = cv::imread(file.path());
            //cout << "Camera image size = " << inputImage.size() << endl;
            cv::imshow("input Image", inputImage);
            cv::waitKey(100);
            
        }
    }
}