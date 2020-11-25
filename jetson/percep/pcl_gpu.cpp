#include "pcl.hpp"
#include "perception.hpp"

/* --- PCL GPU Includes --- */
#include <pcl/gpu/octree/octree.hpp>
#include <pcl/gpu/containers/device_array.hpp>
#include <pcl/gpu/segmentation/gpu_extract_clusters.h>
#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>

//Filters points with values beyond certain threshold
void PCL::PassThroughFilter(){}

//Finds the ground plane
void PCL::RANSACSegmentation(string type){}

/* --- Copy Point Cloud --- */
//Converts a point cloud from point XYZRGB to XYZ
void copyPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr & from, pcl::PointCloud<pcl::PointXYZ>::Ptr & to){
    to->width = from->width;
    to->height = from->height;
    to->is_dense = from->is_dense;
    to->resize(from->width*from->height);

    auto toit = to->points.begin();
    for( auto fromit : from->points){
        toit->x = fromit.x;
        toit->y = fromit.y;
        toit->z = fromit.z;
        toit++;
    }
   
}

/* --- GPU Euclidian Cluster Extraction --- */
//Creates a KdTree structure from point cloud
//Use this tree to traverse point cloud and create vector of clusters
//Return vector of clusters
//Code based on example from: https://tinyurl.com/y62jxrz8
//Source: https://rb.gy/qvjati
void PCL::EuclidianClusterExtraction(std::vector<pcl::PointIndices> &cluster_indices) {
    #if PERCEPTION_DEBUG
        pcl::ScopeTime t ("GPU Cluster Extraction");
    #endif
    pcl::PointCloud<pcl::PointXYZ>::Ptr pt_cloud_ptr_gpu (new pcl::PointCloud<pcl::PointXYZ>);
    copyPointCloud(pt_cloud_ptr, pt_cloud_ptr_gpu);
    pcl::gpu::Octree::PointCloud cloud_device;
    
    cloud_device.upload(pt_cloud_ptr_gpu->points);

    pcl::gpu::Octree::Ptr octree_device (new pcl::gpu::Octree);
    octree_device->setCloud(cloud_device);
    octree_device->build();

    
    pcl::gpu::EuclideanClusterExtraction gec;
    gec.setClusterTolerance (0.02); // 2cm
    gec.setMinClusterSize (100);
    gec.setMaxClusterSize (25000);
    gec.setSearchMethod (octree_device);
    gec.setHostCloud( pt_cloud_ptr_gpu);
    gec.extract (cluster_indices);

}


//Finds the four corners of the clustered obstacles
void PCL::FindInterestPoints(std::vector<pcl::PointIndices> &cluster_indices, std::vector<std::vector<int>> &interest_points){}

//Finds a clear path given the obstacle corners
double PCL::FindClearPath(std::vector<std::vector<int>> interest_points,
                    shared_ptr<pcl::visualization::PCLVisualizer> viewer){}

//Determines whether the input path is obstructed
bool PCL::CheckPath(std::vector<std::vector<int>> interest_points,
        shared_ptr<pcl::visualization::PCLVisualizer> viewer,
        std::vector<int> &obstacles, compareLine leftLine, compareLine rightLine){}

//Main function that runs the above 
obstacle_return PCL::pcl_obstacle_detection(shared_ptr<pcl::visualization::PCLVisualizer> viewer){}

/* --- Create Visualizer --- */
//Creates a point cloud visualizer
shared_ptr<pcl::visualization::PCLVisualizer> PCL::createRGBVisualizer() {
    // Open 3D viewer and add point cloud
    shared_ptr<pcl::visualization::PCLVisualizer> viewer(
      new pcl::visualization::PCLVisualizer("PCL ZED 3D Viewer")); //This is a smart pointer so no need to worry ab deleteing it
    viewer->setBackgroundColor(0.12, 0.12, 0.12);
    pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(pt_cloud_ptr);
    viewer->addPointCloud<pcl::PointXYZRGB>(pt_cloud_ptr, rgb);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1.5);
    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();
    viewer->setCameraPosition(0,0,-800,0,-1,0);
    return (viewer);
}

/* --- Update --- */
//Cleares and resizes cloud for new data
void PCL::update() {
    pt_cloud_ptr->clear();
    pt_cloud_ptr->points.resize(cloudArea);
    pt_cloud_ptr->width = PT_CLOUD_WIDTH;
    pt_cloud_ptr->height = PT_CLOUD_HEIGHT;
    std::cerr << "Width: " << pt_cloud_ptr->width<<std::endl;
    std::cerr << "Height: "<< pt_cloud_ptr->height<<"\n";
}
