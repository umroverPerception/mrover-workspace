#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(void)
{
  pcl::PointCloud<pcl::PointXYZI>::Ptr to_show(new pcl::PointCloud<pcl::PointXYZI>);
  boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("viewer"));
  viewer->setBackgroundColor (0, 0, 0);
  viewer->addCoordinateSystem (1.0f);
  pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> h1(to_show, 255, 0, 0);
  viewer->addPointCloud(to_show, h1, "h1");
  while(!viewer->wasStopped()){
    viewer->spinOnce(100);
  }
}