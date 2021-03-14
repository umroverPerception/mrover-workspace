#include <iostream>
#include <pcl/common/common_headers.h>
//#include <pcl/gpu/segmentation/impl/gpu_extract_clusters.hpp>
#include <pcl/cuda/sample_consensus/sac.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/common/common_headers.h>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/common/time.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/PointIndices.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/kdtree/kdtree.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <string>
//#include <pcl/visualization/pcl_visualizer.h>


using namespace std;


int gpuRANSAC() {
  //  pcl::cuda::SACSegmentation<pcl::PointXYZRGB> seg;

    return 0;
}


int main() {
    string type = "blue";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud_ptr;

    
    pcl::SACSegmentation<pcl::PointXYZRGB> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(400);
    seg.setDistanceThreshold(100); //Distance in mm away from actual plane a point can be
    // to be considered an inlier
    seg.setAxis(Eigen::Vector3f(0, 0, 1)); //Looks for a plane along the Z axis
    double segmentation_epsilon = 45; //Max degree the normal of plane can be from Z axis
    seg.setEpsAngle(pcl::deg2rad(segmentation_epsilon));

    //Objects where segmented plane is stored
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    
    
    seg.setInputCloud(pt_cloud_ptr);
    
    
    seg.segment(*inliers, *coefficients);

    /*
    if(type == "blue"){
        for(int i = 0; i < (int)inliers->indices.size(); i++){
        pt_cloud_ptr->points[inliers->indices[i]].r = 0;
        pt_cloud_ptr->points[inliers->indices[i]].g = 0;
        pt_cloud_ptr->points[inliers->indices[i]].b = 255;
        }
    }
    else {
        //Creates object that filters out identified points
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        extract.setInputCloud(pt_cloud_ptr);
        extract.setIndices(inliers);
        extract.setNegative(true); //Controls whether chosen indices are retained or removed
        extract.filter(*pt_cloud_ptr);
    }*/
    

}