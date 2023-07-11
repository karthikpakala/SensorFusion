
#ifndef TOOLS_H
#define TOOLS_H

#include "Lidar.h"
#include "Camera.h"
#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace Tooling
{
struct Color
{
    float r,g,b;

    Color(float setR, float setG, float setB) 
        : r(setR), g(setG), b(setB) 
    {}
};
enum CameraAngle
{
    XY, TopDown, Side, FPS
};

class Tools
{
    public:

        //enum CameraAngle
        //{
        //    XY, TopDown, Side, FPS
        //};


        //void initCamera(pcl::visualization::PCLVisualizer *pclViewer, CameraAngle cameraAngle);

        //void pclViewer(pcl::PointCloud<LidarPoint> &cloud);

        void initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer);

        void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color = Color(-1,-1,-1));

};
} // namespace Tooling
#endif