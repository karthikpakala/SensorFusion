#include "Lidar.h"

using namespace std;

// Default Constructor.
LidarProcessing::Lidar::Lidar(){}

float LidarProcessing::Lidar::distThreshold = 0.261;
int LidarProcessing::Lidar::numOfIterations = 250;
// Lidar Constructor
LidarProcessing::Lidar::Lidar(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    // Set Point cloud to the point cloud object
    // TODO: Could possibly remove this as the point cloud will be set in the readPCLData function.
    setPointCloud(cloud);
}
/*
LidarProcessing::Lidar::~Lidar()
{
    delete cloud;
    delete viewer;
    delete 
}
*/
// Set input cloud to be processed 
void LidarProcessing::Lidar::setPointCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr cloud)
{
    pointCloud = cloud;
}

pcl::PointCloud<pcl::PointXYZI>::Ptr const LidarProcessing::Lidar::getPointCloud()
{
    return pointCloud;
}

// read PCL file from the file file. 
pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcessing::Lidar::readPCLDataFile(std::string inputFile)
{   
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZI>);

    std::fstream input(inputFile.c_str(), ios::in | ios::binary);
    if(!input.good())
    {
        std::cerr <<"Input file not loaded" << endl;
    }
    input.seekg(0, ios::beg);

    for(int i = 0; input.good() && !input.eof(); i++)
    {
        pcl::PointXYZI point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud->push_back(point);
    }

    // Use setter to set the the point cloud to this class. 
    // setPointCloud(cloud);
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+inputFile << std::endl;

    return cloud;
}

// crop Lidar Points to capture only pints in the camera image frame.
pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcessing::Lidar::cropLidarPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{

    // remove Lidar points based on distance properties
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempLidarPoints (new pcl::PointCloud<pcl::PointXYZI>);
    
    int index = 0;
    for(auto it = cloud->points.begin(); it < cloud->points.end(); ++it)
    {

        if((*it).x >= minX && (*it).x <= maxX && abs((*it).y) <= maxY && (*it).z >= minZ && (*it).z <= maxZ && (*it).intensity >= minR)
        {
            tempLidarPoints->points.push_back(*it);
        }
    }

    cloud->points = tempLidarPoints->points;
    return cloud;
}

// Filter point cloud to remove unnecessary points
pcl::PointCloud<pcl::PointXYZI>::Ptr LidarProcessing::Lidar::filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint)
{
    pcl::VoxelGrid<pcl::PointXYZI> vg;
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudFiltered (new pcl::PointCloud<pcl::PointXYZI>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Filter out region that is not relevant
    pcl::PointCloud<pcl::PointXYZI>::Ptr cloudRegion (new pcl::PointCloud<pcl::PointXYZI>);
    
    pcl::CropBox<pcl::PointXYZI> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<pcl::PointXYZI> roof(true);
    roof.setMin(Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Vector4f(2.6, 1.7, -4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
    for(int point : indices)
    {
        inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<pcl::PointXYZI> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    return cloudRegion;


}

// segment the point cloud to define points corresponding to the road and points corresponding tot the objects. 
std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> LidarProcessing::Lidar::ransacPlaneSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedClouds;

    std::unordered_set<int> tempInliers;

    int numOfIterations = 250;
   // float distThreshold = 0.261; 
    while(numOfIterations--)
    {
        // Create a plane using random points in the cloud
        std::unordered_set<int> inliers;

        while(inliers.size() < 3)
        {
			inliers.insert(rand()%(cloud->points.size()));
		}

        // Create a plane using 3 points from the 
        float x1, y1, z1, x2, y2, z2, x3, y3, z3;

        auto itr = inliers.begin();

        x1 = cloud->points.at(*itr).x;
        y1 = cloud->points.at(*itr).y;
        z1 = cloud->points.at(*itr).z;
        
        // Increment iterator to the next point in the cloud
        itr++;
        x2 = cloud->points.at(*itr).x;
        y2 = cloud->points.at(*itr).y;
        z2 = cloud->points.at(*itr).z;

        // Increment iterator to the next point in the cloud
        itr++;
        x3 = cloud->points.at(*itr).x;
        y3 = cloud->points.at(*itr).y;
        z3 = cloud->points.at(*itr).z;

        // create the constants for the plane equation

        float A = ((y2-y1)*(z3=z1) - (z2-z1)*(y3-y1));
        float B = ((z2-z1)*(x3-x1) - (x2-x1)*(z3-z1));
        float C = ((x2-x1)*(y3-y1) - (y2-y1)*(x3-x1));
        float D = -(A*x1 + B*y1 + C*z1);

        float denominator = sqrt(A*A + B*B + C*C);

        for(int i = 0; i < cloud->points.size(); i++)
        {
            float distance = fabs(A*(cloud->points.at(i).x) + B*(cloud->points.at(i).y) + C*(cloud->points.at(i).z) + D)/denominator;

            if(distance <= distThreshold)
            {
                inliers.insert(i);
            }
        }
        
        if(inliers.size() > tempInliers.size())
        {
            tempInliers = inliers;
        }
    }

    pcl::PointCloud<pcl::PointXYZI>::Ptr inlierCloud (new pcl::PointCloud<pcl::PointXYZI>);
    pcl::PointCloud<pcl::PointXYZI>::Ptr outlierCloud (new pcl::PointCloud<pcl::PointXYZI>);

    for(int i = 0; i < cloud->points.size(); i++)
    {
        if(tempInliers.count(i))
        {
            inlierCloud->points.push_back(cloud->points.at(i));
        }
        else
        {
            outlierCloud->points.push_back(cloud->points.at(i));
        }
    }

    segmentedClouds.first = inlierCloud;
    segmentedClouds.second = outlierCloud;
    
    return segmentedClouds; 
}

// Apply clustering on the point cloud to classify all the points in the cloud and create various objects out of the cloud. 
std::vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> LidarProcessing::Lidar::Clustering(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float distThreshold, int minCount, int maxCount)
{
    vector<pcl::PointCloud<pcl::PointXYZI>::Ptr> clusteredObjects;

    pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);

    tree->setInputCloud(cloud);

    vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZI> ecObject;
    ecObject.setClusterTolerance(distThreshold);
    ecObject.setMinClusterSize(minCount);
    ecObject.setMaxClusterSize(maxCount);
    ecObject.setSearchMethod(tree);
    ecObject.setInputCloud(cloud);
    ecObject.extract(clusterIndices);

    for(auto& indice : clusterIndices)
    {
        pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZI>);
        for(const auto& idx : clusterIndices)
        {
            //cloud_cluster->push_back((*cloud).idx);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        cout << "Point Cloud representing the cluster: " << cloud_cluster->size() << endl;
    }
    return clusteredObjects;
}

