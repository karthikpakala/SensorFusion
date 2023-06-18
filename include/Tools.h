
#ifndef TOOLS_H
#define TOOLS_H

#include "Lidar.h"
#include "Camera.h"
//#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>


class Tools
{
    public:

        enum CameraAngle
        {
            XY, TopDown, Side, FPS
        };

        //void initCamera(pcl::visualization::PCLVisualizer *pclViewer, CameraAngle cameraAngle);

        void pclViewer(pcl::PointCloud<LidarPoint> &cloud);

};
#endif