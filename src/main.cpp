#include <iostream>
#include <filesystem>
#include "Lidar.h"
#include "Radar.h"
#include "Camera.h"
#include "Calibration.h"
#include "Tools.h"

using namespace std;


int main(int argv, char **argc)
{
    int dataBufferSize = 2;       // no. of images which are held in memory (ring buffer) at the same time
    //vector<DataFrame> dataBuffer; // list of data frames which are held in memory at the same time

    // Initialize calibration object
    Calibration calibration;

    // Tools Object
    Tools tools;

    //calibration.initializeMatrices();

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
    pcl::PointCloud<LidarPoint> cloud;

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
        std::set<filesystem::path> sorted_by_name;

        for(auto& file : filesystem::directory_iterator(fullPCLFolderPath))
        {
            sorted_by_name.insert(file.path());
        }
        // Load Lidatr data into buffer. 
        for(auto& fileName : sorted_by_name)
        {
            // Initialize a Lidar Object to load ane process the Lidar Data
            Lidar *lidar;
            //lidar.readPCLDataFile(cloud, file.path());
            
            //  TODO: Read files in ascending order to read them coprrectly.
            pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);
            cout << "File Path = " << fileName.c_str() << endl;

            
            cloud = lidar->readPCLDataFile(fileName.c_str());

            for(int i = 0; i < cloud->points.size(); i++)
            {
                //cout << " X = " << cloud->points.at(i).x << " Y = " << cloud->points.at(i).y << " Z = " << cloud->points.at(i).z << " Intensity = " << cloud->points.at(i).intensity << endl;
            }

            cout << "Lidar PCD size = " << cloud->points.size() << endl;

            // Write code to process lidar points
            //lidar->cropLidarPoints(cloud);
            // cout << "Lidar Points after cropping = " << cloud->points.size() << endl;

            pcl::PointCloud<pcl::PointXYZI>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZI>);
            // Filter point clouds
            filteredCloud = lidar->filterCloud(cloud, 0.1, Vector4f(-20, -6 , -3 , 1), Vector4f(25, 6.5, 3, 1));

            cout << "Filtered cloud Size = " << filteredCloud->points.size() << endl;

            float distThreshold = 0.3;
            int numOfIterations = 200;
            //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedClouds (new std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>);
            std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedClouds  = lidar->ransacPlaneSegmentation(filteredCloud, numOfIterations, distThreshold);

            cout << "inlier Cloud size = " << segmentedClouds.first->points.size() << " | outlier Cloud size = " << segmentedClouds.second->points.size() << endl;
            //pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud (new pcl::PointCloud<pcl::PointXYZI>);
            //pcl::PointCloud<pcl::PointXYZI>::Ptr outlierCloud (new pcl::PointCloud<pcl::PointXYZI>);

            //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud = lidar->ransacPlaneSegmentation(filteredCloud, numOfIterations, distThreshold);
           // segmentedCloud  = lidar->ransacPlaneSegmentation(filteredCloud, numOfIterations, distThreshold);
            //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud (new )
            //std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedCloud (new (std::pair<pcl::PointCloud<pcl::PointXYZI>, pcl::PointCloud<pcl::PointXYZI>));
            //cout << "Segmented cloud sizes | inliers size =" << segmentedClouds.first->points.size() << segmentedClouds.second->points.size() << endl;

        }

        // Load input image into the buffer. 
        for(auto& file : filesystem::directory_iterator(fullImageFolderPath))
        {
            //inputImage = cv::imread(file.path());
            //cout << "Camera image size = " << inputImage.size() << endl;
            //cv::imshow("input Image", inputImage);
            //cv::waitKey(100);
            
        }
    }
}