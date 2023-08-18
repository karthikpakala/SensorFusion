#include "Lidar.h"

//using namespace std;
using namespace LidarProcessing;


// PointCloud setter
template<typename PointT>
void LidarProcessing::Lidar<PointT>::setPointCloud(typename pcl::PointCloud<PointT>::Ptr &pointCloud)
{
  pointCloud = pointCloud;
}

// Point Cloud getter
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr LidarProcessing::Lidar<PointT>::getPointCloud()
{
  return pointCloud;
} 


template<typename PointT>
void LidarProcessing::Lidar<PointT>::setNumberOfIterations(int &numOfIterations)
{
  numberOfIterations = numOfIterations;
}

template<typename PointT>
int LidarProcessing::Lidar<PointT>::getNumberOfIterations()
{
  return numberOfIterations;
}

template<typename PointT>
void LidarProcessing::Lidar<PointT>::setDistanceThreshold(float &distThreshold)
{
  distanceThreshold = distThreshold;
}

template<typename PointT>
float LidarProcessing::Lidar<PointT>::getDistanceThreshold()
{
  return distanceThreshold;
}



// read PCL file from the file file. 
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr LidarProcessing::Lidar<PointT>::readPCLDataFile(std::string inputFile)
{   
    
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);

    std::fstream input(inputFile.c_str(), std::ios::in | std::ios::binary);
    if(!input.good())
    {
        std::cerr <<"Input file not loaded" << std::endl;
    }
    input.seekg(0, std::ios::beg);

    for(int i = 0; input.good() && !input.eof(); i++)
    {
        PointT point;
        input.read((char *) &point.x, 3*sizeof(float));
        input.read((char *) &point.intensity, sizeof(float));
        cloud->push_back(point);
    }

    // Use setter to set the the point cloud to this class. 
    setPointCloud(cloud);
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+inputFile << std::endl;

    return cloud;
}

// Filter point cloud to remove unnecessary points

template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr LidarProcessing::Lidar<PointT>::filterCloud(typename pcl::PointCloud<PointT>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint)
{
    pcl::VoxelGrid<PointT> vg;
    typename pcl::PointCloud<PointT>::Ptr cloudFiltered(
        new pcl::PointCloud<PointT>);
    vg.setInputCloud(cloud);
    vg.setLeafSize(filterRes, filterRes, filterRes);
    vg.filter(*cloudFiltered);

    // Filter out region that is not relevant
    typename pcl::PointCloud<PointT>::Ptr cloudRegion(
        new typename pcl::PointCloud<PointT>);

    pcl::CropBox<PointT> region(true);
    region.setMin(minPoint);
    region.setMax(maxPoint);
    region.setInputCloud(cloudFiltered);
    region.filter(*cloudRegion);

    std::vector<int> indices;

    pcl::CropBox<PointT> roof(true);
    roof.setMin(Vector4f(-1.5, -1.7, -1, 1));
    roof.setMax(Vector4f(2.6, 1.7, -4, 1));
    roof.setInputCloud(cloudRegion);
    roof.filter(indices);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    for (int point : indices) 
    {
      inliers->indices.push_back(point);
    }

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloudRegion);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*cloudRegion);

    return cloudRegion;
  }

// segment the point cloud to define points corresponding to the road and points corresponding tot the objects. 


template<typename PointT>
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> LidarProcessing::Lidar<PointT>::ransacPlaneSegmentation(typename pcl::PointCloud<PointT>::Ptr &cloud)
{

    std::pair<typename pcl::PointCloud<PointT>::Ptr,
              typename pcl::PointCloud<PointT>::Ptr>
        segmentedClouds;

    std::unordered_set<int> tempInliers;

    int numOfIterations = 100;
    float distThreshold = 0.350;
    while (numOfIterations--) 
    {
      // Create a plane using random points in the cloud
      std::unordered_set<int> inliers;

      while (inliers.size() < 3) 
      {
        inliers.insert(rand() % (cloud->points.size()));
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
      float A = ((y2 - y1) * (z3 = z1) - (z2 - z1) * (y3 - y1));
      float B = ((z2 - z1) * (x3 - x1) - (x2 - x1) * (z3 - z1));
      float C = ((x2 - x1) * (y3 - y1) - (y2 - y1) * (x3 - x1));
      float D = -(A * x1 + B * y1 + C * z1);

      float denominator = sqrt(A * A + B * B + C * C);

      for (int i = 0; i < cloud->points.size(); i++) 
      {
        float distance =
            fabs(A * (cloud->points.at(i).x) + B * (cloud->points.at(i).y) +
                 C * (cloud->points.at(i).z) + D) / denominator;

        if (distance <= distThreshold) 
        {
          inliers.insert(i);
        }
      }

      if (inliers.size() > tempInliers.size()) 
      {
        tempInliers = inliers;
      }

    }

    typename pcl::PointCloud<PointT>::Ptr inlierCloud(new typename pcl::PointCloud<PointT>);
    typename pcl::PointCloud<PointT>::Ptr outlierCloud(new typename pcl::PointCloud<PointT>);

    for (int i = 0; i < cloud->points.size(); i++) 
    {
      if (tempInliers.count(i)) 
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

/*
// crop Lidar Points to capture only pints in the camera image frame.
template<typename PointT>
typename pcl::PointCloud<PointT>::Ptr LidarProcessing::Lidar<PointT>::cropLidarPoints(typename pcl::PointCloud<PointT>::Ptr &cloud)
{

    // remove Lidar points based on distance properties
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
    typename pcl::PointCloud<PointT>::Ptr tempLidarPoints (new pcl::PointCloud<PointT>);
    
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
*/


// Apply clustering on the point cloud to classify all the points in the cloud and create various objects out of the cloud. 
template<typename PointT>
std::vector<typename pcl::PointCloud<PointT>::Ptr> LidarProcessing::Lidar<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr &cloud, float distThreshold, int minCount, int maxCount)
{
    std::vector<typename pcl::PointCloud<PointT>::Ptr> clusteredObjects;

    typename pcl::search::KdTree<PointT>::Ptr tree (new typename pcl::search::KdTree<PointT>);

    tree->setInputCloud(cloud);

    std::vector<pcl::PointIndices> clusterIndices;
    pcl::EuclideanClusterExtraction<PointT> ecObject;
    ecObject.setClusterTolerance(distThreshold);
    ecObject.setMinClusterSize(minCount);
    ecObject.setMaxClusterSize(maxCount);
    ecObject.setSearchMethod(tree);
    ecObject.setInputCloud(cloud);
    ecObject.extract(clusterIndices);

    for(auto& indice : clusterIndices)
    {
        typename pcl::PointCloud<PointT>::Ptr cloud_cluster (new pcl::PointCloud<PointT>);
        for(const auto& idx : clusterIndices)
        {
            //cloud_cluster->push_back((*cloud).idx);
        }
        cloud_cluster->width = cloud_cluster->size();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;

        std::cout << "Point Cloud representing the cluster: " << cloud_cluster->size() << std::endl;
    }
    return clusteredObjects;
}


template class Lidar<pcl::PointXYZI>;
