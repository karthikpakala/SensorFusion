#include "Lidar.h"

using namespace std;
/*
void Lidar::readPCLDataFile(pcl::PointCloud<LidarPoint> &cloud, std::string inputFile)
{   
    // Number of Lidar Points in a Frame
    unsigned long numOfPoints = 1000000;
    unsigned long bufferSize = 0;

    // Allocate memory to hold the data
    float *data = (float*)malloc(numOfPoints*sizeof(float));

    //Lidar variable Pointers
    float *px = data + 0;
    float *py = data + 1;
    float *pz = data + 2;
    float *pi = data + 3;

    // Create a file stream to read data into
    FILE *stream;

    // Open the file stream and read the input file
    stream = fopen(inputFile.c_str(), "rb");
    numOfPoints = fread(data, sizeof(float), numOfPoints, stream) / 4;

    // iterate through all the data and create a vector of Lidar points
    for(int32_t i = 0; i < numOfPoints; i++)
    {
        LidarPoint lidarPoint;
        lidarPoint.x_coordinate = *px;
        lidarPoint.y_coordinate = *py;
        lidarPoint.z_coordinate = *pz;
        lidarPoint.intensity = *pi;
        lidarPoint.index = i + 1;

        cloud.push_back(lidarPoint);

        //cout << "Lidar Point [X = " << cloud.at(i).x_coordinate << " Y = " << cloud.at(i).y_coordinate << "  Z = " << cloud.at(i).z_coordinate << " I = " << cloud.at(i).intensity << endl;

        px+=4;
        py+=4;
        pz+=4;
        pi+=4;
    }

    fclose(stream);
}
*/

pcl::PointCloud<pcl::PointXYZI>::Ptr Lidar::readPCLDataFile(std::string inputFile)
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
        //std::cerr << " Data = X = " << cloud->points.at(i).x << " Y = " << cloud->points.at(i).y << " Z = " << cloud->points.at(i).z << " Intensity = " << cloud ->points.at(i).intensity << endl;
    }
    std::cerr << "Loaded " << cloud->points.size () << " data points from "+inputFile << std::endl;

    return cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr Lidar::cropLidarPoints(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud)
{

    // remove Lidar points based on distance properties
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
    //cout << "after limit values definition" << endl;
    pcl::PointCloud<pcl::PointXYZI>::Ptr tempLidarPoints (new pcl::PointCloud<pcl::PointXYZI>);
    //cout << "after temp variable definition" << endl;
    
    int index = 0;
    for(auto it = cloud->points.begin(); it < cloud->points.end(); ++it)
    {
        //cout << "Cloud size = " << cloud->points.size() << endl;
        //cout << "Inside crop points for loop" << endl;
        if((*it).x >= minX && (*it).x <= maxX && abs((*it).y) <= maxY && (*it).z >= minZ && (*it).z <= maxZ && (*it).intensity >= minR)
        {
            //cout << "inside if loop" << endl;
            tempLidarPoints->points.push_back(*it);
        }
        //cout << "after if loop in for loop" << endl;
        //cout <<"Index = " << index++ << endl;
    }
    //cout << "after for loop" << endl;
    //cloud->points.clear();
    //cout << "after clear cloud" << endl;
    cloud->points = tempLidarPoints->points;
    //cout << "cloud points size = " << cloud->points.size() << endl;
    return cloud;
}


pcl::PointCloud<pcl::PointXYZI>::Ptr Lidar::filterCloud(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, float filterRes, Vector4f minPoint, Vector4f maxPoint)
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


std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> Lidar::ransacPlaneSegmentation(pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud, int maxIterations, float distanceThreshold)
{
    std::pair<pcl::PointCloud<pcl::PointXYZI>::Ptr, pcl::PointCloud<pcl::PointXYZI>::Ptr> segmentedClouds;

    std::unordered_set<int> tempInliers;
    while(maxIterations--)
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

        //for(auto itr = cloud->points.begin(); itr < cloud->points.end(); ++itr)
        for(int i = 0; i < cloud->points.size(); i++)
        {
            //float distance = fabs(A*((*itr).x) + B*((*itr).y) + C*((*itr).z) + D)/denominator;
            float distance = fabs(A*(cloud->points.at(i).x) + B*(cloud->points.at(i).y) + C*(cloud->points.at(i).z) + D)/denominator;

            if(distance <= distanceThreshold)
            {
                //inliers.insert(*itr);
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

    //for(auto itr = cloud->points.begin(); cloud->points.end(); ++itr)
    for(int i = 0; i < cloud->points.size(); i++)
    {
        //if(tempInliers.count(*itr))
        if(tempInliers.count(i))
        {
            //inlierCloud->points.push_back(*itr);
            inlierCloud->points.push_back(cloud->points.at(i));
        }
        else
        {
            //outlierCloud->points.push_back(*itr);
            outlierCloud->points.push_back(cloud->points.at(i));
        }
    }

    segmentedClouds.first = inlierCloud;
    segmentedClouds.second = outlierCloud;
    
    return segmentedClouds; 
}

