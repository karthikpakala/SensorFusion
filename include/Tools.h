
#ifndef TOOLS_H
#define TOOLS_H

#include <pcl/visualization/cloud_viewer.h>
//#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/pcl_visualizer.h>

namespace Tooling {

// Color definition for Lidar Preview
struct Color 
{
  float r, g, b;

  Color(float setR, float setG, float setB) : r(setR), g(setG), b(setB) {}
};

// Camera Angle Options
enum CameraAngle 
{
  XY, TopDown, Side, FPS 
};


class Tools {
private:
  static Tools* toolsInstance;

  // Default Constructor
  Tools() 
  {}

public:
  Tools(const Tools &tools) = delete; // Delete Copy Constructor
  Tools& operator=(const Tools &tools) = delete; // Delete Copy assignment Operator
  Tools(Tools &&tools) = delete; // Delete Move Constructor
  Tools& operator=(Tools &&tools) = delete; // Delete Move assignment operator

  // Create a static object for Tools class.
  static Tools* getInstance()
  {
    if(toolsInstance == NULL)
    {
      toolsInstance = new Tools(); 
      return toolsInstance;
    }
    else
    {
      return toolsInstance;
    }
  }

  void setToolInstance(Tools* inputToolInstance)
  {
    toolsInstance = inputToolInstance;
  }

  // Initialize Camera parameters
  void initCamera(CameraAngle setAngle,
                  pcl::visualization::PCLVisualizer::Ptr &viewer);

  void renderPointCloud(pcl::visualization::PCLVisualizer::Ptr &viewer,
                        pcl::PointCloud<pcl::PointXYZI>::Ptr &cloud,
                        std::string name, Color color = Color(-1, -1, -1));
};
} // namespace Tooling
#endif