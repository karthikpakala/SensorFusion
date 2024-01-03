#include "Lidar.h"

//using namespace std;
using namespace LidarProcessing;
using namespace Tooling;

// Create Tools object
Tools *tools;

// Default Constructor
template<typename PointT>
LidarProcessing::Lidar<PointT>::Lidar() : pointCloud (new pcl::PointCloud<pcl::PointXYZI>)
{
}

// Constructor
template<typename PointT>
LidarProcessing::Lidar<PointT>::Lidar(typename pcl::PointCloud<PointT>::Ptr &inputCloud)
{
  std::cout << "Inside Lidar constructor" << std::endl;
  pointCloud = std::move(inputCloud);
}

// Destructor
template<typename PointT>
LidarProcessing::Lidar<PointT>::~Lidar() 
{
  //delete pointCloud;
  //std::cout << "Lidar class object destroyed" << std::endl;
}

// Copy Constructor
template<typename PointT>
LidarProcessing::Lidar<PointT>::Lidar(const Lidar<PointT> &lidarObject)
{
  this->pointCloud = lidarObject.pointCloud;
  this->numberOfIterations = lidarObject.numberOfIterations;
  this->distanceThreshold = lidarObject.distanceThreshold;
}

// Copy Assignment Operator
template<typename PointT>
LidarProcessing::Lidar<PointT> &LidarProcessing::Lidar<PointT>::operator=(const Lidar<PointT> &lidarObject)
{
  if(this == &lidarObject)
  {
    return *this;
  }

  pointCloud = lidarObject.pointCloud;
  numberOfIterations = lidarObject.numberOfIterations;
  distanceThreshold = lidarObject.distanceThreshold;
  // initialize class variables
  return *this;
}

// Move constructor
template<typename PointT>
LidarProcessing::Lidar<PointT>::Lidar(Lidar<PointT> &&lidarObject) noexcept
{}

// Move assignment operator
template<typename PointT>
LidarProcessing::Lidar<PointT> &LidarProcessing::Lidar<PointT>::operator=(Lidar<PointT> &&lidarObject) noexcept
{
  if(this == &lidarObject)
  {
    return *this;
  }
  // initialize variables
  return *this;
}


// PointCloud setter
template<typename PointT>
void LidarProcessing::Lidar<PointT>::setPointCloud(typename pcl::PointCloud<PointT>::Ptr &inputPointCloud)
{
   pointCloud = std::move(inputPointCloud);
  //processPointCloud(pointCloud);
}

template<typename PointT>
void LidarProcessing::Lidar<PointT>::readFileHelper(std::fstream &input, typename pcl::PointCloud<PointT>::Ptr &cloud)
{
    PointT point;
    input.read((char *) &point.x, 3*sizeof(float));
    input.read((char *) &point.intensity, sizeof(float));
    cloud->push_back(point);
}

// read PCL file from the file file.
// TODO : Update function to populate class point cloud object instead of using a sepaerate object 
template<typename PointT>
//typename pcl::PointCloud<PointT>::Ptr LidarProcessing::Lidar<PointT>::readPCLDataFile(std::string inputFile, pcl::visualization::PCLVisualizer::Ptr &viewer)
void LidarProcessing::Lidar<PointT>::readPCLDataFile(std::string inputFile, pcl::visualization::PCLVisualizer::Ptr &viewer)

{   
    //std::lock_guard<std::mutex> lock(lidarDataLock);
    //lidarDataLock.lock();
    //std::cout<< " File Name = " << inputFile << std::endl;
    
    // typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    std::mutex pointCloudMutex;
    pointCloudMutex.lock();
    typename pcl::PointCloud<PointT>::Ptr cloud (new pcl::PointCloud<PointT>);
    pointCloudMutex.unlock();

    //auto startTime =  std::chrono::steady_clock::now();
    
    std::mutex fileStreamMutex;
    fileStreamMutex.lock();
    std::fstream input(inputFile.c_str(), std::ios::in | std::ios::binary);
    fileStreamMutex.unlock();

    if(!input.good())
    {
        std::cerr <<"Input file not loaded" << std::endl;
    }
    input.seekg(0, std::ios::beg);

    std::vector<std::future<void>> futures;
    auto readTimeStart = std::chrono::steady_clock::now();
    std::mutex readFileMutex;
    for(int i = 0; input.good() && !input.eof(); i++)
    {
      std::lock_guard<std::mutex> lock(readFileMutex);
      readFileHelper(input, cloud);

      //futures.emplace_back(std::async(std::launch::deferred, &LidarProcessing::Lidar<PointT>::readFileHelper, this, std::ref(input), std::ref(cloud)));
    }

    auto readTimeEnd = std::chrono::steady_clock::now();
    auto totalReadTime = std::chrono::duration_cast<std::chrono::milliseconds>(readTimeEnd - readTimeStart);

    std::cout << "Total File Read Time = " << totalReadTime.count() << std::endl;
    for(auto &ftr : futures)
    {
      ftr.wait();
    }

    //auto endTime = std::chrono::steady_clock::now();

    //auto elapsedTime = std::chrono::duration_cast<std::chrono::milliseconds> ( endTime - startTime);
    //std::cout << "Time elapsed readPCLFile = " << elapsedTime.count()<< std::endl;
    std::cerr << "Loaded " << cloud->points.size () << " data points from cloud "+inputFile << std::endl;

    auto startTimeprocess = std::chrono::steady_clock::now();

    setPointCloud(cloud);
    processPointCloud(pointCloud, viewer);

    auto endTimeprocess = std::chrono::steady_clock::now();
    auto elapsedTimeprocess = std::chrono::duration_cast<std::chrono::milliseconds> (endTimeprocess - startTimeprocess);
    std::cout << "Point Cloud set time = " << elapsedTimeprocess.count() << std::endl;
    //lidarDataLock.unlock();
    //return pointCloud;
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

template<typename PointT> 
void LidarProcessing::Lidar<PointT>::processPointCloud(typename pcl::PointCloud<PointT>::Ptr &inputCloud, pcl::visualization::PCLVisualizer::Ptr &viewer)
{
  // Step 1 : Filter Point Cloud
  typename pcl::PointCloud<PointT>::Ptr filteredCloud (new pcl::PointCloud<pcl::PointXYZI>);
  filteredCloud = filterCloud(inputCloud, 0.1, Vector4f(-20, -6, -3, 1), Vector4f(25, 6.5, 3, 1));
  std::cout << "FilteredCloud point count = " << filteredCloud->points.size() << std::endl;

  // Step 2 : RANSAC Segmentation
  std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr>
              segmentedClouds = ransacPlaneSegmentation(filteredCloud, viewer);


// Step 3: Clustering
//std::vector<typename pcl::PointCloud<PointT>::Ptr> clusters = Clustering(segmentedClouds.first,0.359, 100, 2000);

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
std::pair<typename pcl::PointCloud<PointT>::Ptr, typename pcl::PointCloud<PointT>::Ptr> LidarProcessing::Lidar<PointT>::ransacPlaneSegmentation(typename pcl::PointCloud<PointT>::Ptr &cloud, pcl::visualization::PCLVisualizer::Ptr &viewer)
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

    tools->renderPointCloud(viewer, segmentedClouds.first, "sample cloud",
                                  Color(0, 1, 0));
    tools->renderPointCloud(viewer, segmentedClouds.second,
                                 "object cloud", Color(1, 0, 0));
          // viewer->spin();
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
std::vector<typename pcl::PointCloud<PointT>::Ptr> LidarProcessing::Lidar<PointT>::Clustering(typename pcl::PointCloud<PointT>::Ptr &cloud, float distThreshold, int minCount, int maxCount, pcl::visualization::PCLVisualizer::Ptr &viewer)
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
template class LidarProcessing::Lidar<pcl::PointXYZI>;


