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

/*
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
*/

void Tooling::Tools::initCamera(CameraAngle setAngle, pcl::visualization::PCLVisualizer::Ptr& viewer)
{

    viewer->setBackgroundColor (0, 0, 0);
    
    // set camera position and angle
    viewer->initCameraParameters();
    // distance away in meters
    int distance = 16;
    
    switch(setAngle)
    {
        case XY : viewer->setCameraPosition(-distance, -distance, distance, 1, 1, 0); break;
        case TopDown : viewer->setCameraPosition(0, 0, distance, 1, 0, 1); break;
        case Side : viewer->setCameraPosition(0, -distance, 0, 0, 0, 1); break;
        case FPS : viewer->setCameraPosition(-10, 0, 0, 0, 0, 1);
    }

    if(setAngle!=FPS)
        viewer->addCoordinateSystem (1.0);
}

void Tooling::Tools::renderPointCloud(pcl::visualization::PCLVisualizer::Ptr& viewer, pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud, std::string name, Color color)
{
    /*
    bool bVis = true;
	if(color.r==-1)
	{
		// Select color based off of cloud intensity
		pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud,"intensity");
  		viewer->addPointCloud<pcl::PointXYZI>(cloud, intensity_distribution, name);
	}
	else
	{
		// Select color based off input value
		viewer->addPointCloud<pcl::PointXYZI> (cloud, name);
		viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
	}

	viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, name);
    */

    viewer->getRenderWindow()->GlobalWarningDisplayOff(); // suppress VTK warnings        
    pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> intensity_distribution(cloud, "intensity");
    //viewer->addPointCloud<pcl::PointXYZI> (cloud, intensity_distribution, "sample cloud");
    viewer->addPointCloud<pcl::PointXYZI> (cloud, name);            	
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_COLOR, color.r, color.g, color.b, name);
    viewer->setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, name);
    //viewer->addPointCloudNormals<pcl::PointXYZI, pcl::Normal> (cloud, normals, 10, 0.05, "normals");
    viewer->addCoordinateSystem(1.0);
    //viewer->initCameraParameters();
}
