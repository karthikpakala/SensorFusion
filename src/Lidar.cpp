#include "Lidar.h"

void Lidar::readPCLDataFile(vector<LidarPoint> &lidarPoints, std::string inputFile)
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

        lidarPoints.push_back(lidarPoint);

        //cout << "Lidar Point [X = " << lidarPoints.at(i).x_coordinate << " Y = " << lidarPoints.at(i).y_coordinate << "  Z = " << lidarPoints.at(i).z_coordinate << " I = " << lidarPoints.at(i).intensity << endl;

        px+=4;
        py+=4;
        pz+=4;
        pi+=4;
    }

    fclose(stream);
}

vector<LidarPoint> Lidar::cropLidarPoints(vector<LidarPoint> &lidarPoints)
{

    // remove Lidar points based on distance properties
    float minZ = -1.5, maxZ = -0.9, minX = 2.0, maxX = 20.0, maxY = 2.0, minR = 0.1; // focus on ego lane
    
    vector<LidarPoint> tempLidarPoints;
    for(auto it = lidarPoints.begin(); it < lidarPoints.end(); ++it)
    {
        if((*it).x_coordinate >= minX && (*it).x_coordinate <= maxX && abs((*it).y_coordinate) <= maxY && (*it).z_coordinate >= minZ && (*it).z_coordinate <= maxZ && (*it).intensity >= minR)
        {
            tempLidarPoints.push_back(*it);
        }
    }

    lidarPoints = tempLidarPoints;
    return lidarPoints;
}

