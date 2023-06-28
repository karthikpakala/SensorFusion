#include "Tools.h"

/*
void Tools::initCamera(pcl::visualization::PCLVisualizer *pclViewer, CameraAngle cameraAngle)
{
    // set back ground color
    pclViewer->setBackgroundColor(0,0,0);

    // set camera angle
    pclViewer->initCameraParameters();

    int distance = 20;

    switch (cameraAngle)

    {
        case XY : 
            pclViewer->setCameraPosition(-distance, -distance, distance,1, 1, 0); 
            break;
        case TopDown : 
            pclViewer->setCameraPosition(0, 0, distance, 1, 0, 1); 
            break;
        case Side : 
            pclViewer->setCameraPosition(0, -distance, 0, 0, 0, 1); 
            break;
        case FPS : 
            pclViewer->setCameraPosition(-10, 0, 0, 0, 0, 1);

    }

    if(cameraAngle!=FPS)
    {
        pclViewer->addCoordinateSystem(1.0);
    }
}
*/

void Tools::pclViewer(pcl::PointCloud<LidarPoint> &cloud)
{
    pcl::visualization::PCLVisualizer::Ptr cloudViewer (new pcl::visualization::PCLVisualizer);
    cloudViewer->setBackgroundColor(0,0,0);
    cloudViewer->initCameraParameters();
    cloudViewer->setCameraPosition(-20, -20, 20, 1, 1, 0);
    cloudViewer->addCoordinateSystem(1.0);
    //cloudViewer->addPointCloud(cloud, "Cloud");
    //std::string name = "cloud";
    //cloudViewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
}
