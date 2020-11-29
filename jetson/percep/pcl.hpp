#if OBSTACLE_DETECTION
#pragma once


#include "perception.hpp"
#include "rapidjson/document.h"
using namespace rapidjson;
#include <pcl/common/common_headers.h>

/* --- Compare Line Class --- */
//Functor that indicates where a point is in
//relation to a line in 2D space
class compareLine {
public:
    double m;
    int b;
    
    compareLine(double slope_in, int b_in) : m(slope_in), b(b_in){}

    //Returns 1 if point is above line, 0 if on, -1 if below
    int operator()(int x, int y) {
        double yc = x*m+b;
        if(y > yc)
            return 1;
        else if (y == yc)
            return 0;
        else
            return -1; 
    }
};

class PCL {
    public:

    //Constructor
    PCL() : 
        bearing{0}, distance{0}, detected{false},
        pt_cloud_ptr{new pcl::PointCloud<pcl::PointXYZRGB>} {

        ifstream configFile;
        string configPath = getenv("MROVER_CONFIG");
        configPath += "/config_nav/config.json";
        configFile.open( configPath );
        string config = "";
        string setting;
        while( configFile >> setting )
        {
           config += setting;
        }
        configFile.close();

        mRoverConfig.Parse( config.c_str() );

        #if ZED_SDK_PRESENT
        sl::Resolution cloud_res = sl::Resolution(mRoverConfig["pt_cloud"]]["pt_cloud_width"].GetDouble(), mRoverConfig["pt_cloud"]]["pt_cloud_height"].GetDouble());
        cloudArea = cloud_res.area();
        #else
        cloudArea = mRoverConfig["pt_cloud"]]["pt_cloud_width"].GetInt()*mRoverConfig["pt_cloud"]]["pt_cloud_height"].GetInt();
        std::cerr << cloudArea<< endl;
        #endif

    };

    double bearing;
    double distance;
    bool detected;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr pt_cloud_ptr;
    int cloudArea;
    double HALF_ROVER=mRoverConfig["pt_cloud"]["half_rover"].GetDouble();
    double CENTERX=mRoverConfig["pt_cloud"]["centerx"].GetDouble();

    private:

        //Filters points with values beyond certain threshold
        void PassThroughFilter();
        
        //Clusters nearby points to reduce total number of points
        void DownsampleVoxelFilter();
        
        //Finds the ground plane
        void RANSACSegmentation(string type);
        
        //Clusters nearby points into large obstacles
        void CPUEuclidianClusterExtraction(std::vector<pcl::PointIndices> &cluster_indices);
        
        //Finds the four corners of the clustered obstacles
        void FindInterestPoints(std::vector<pcl::PointIndices> &cluster_indices, std::vector<std::vector<int>> &interest_points);
        
        //Finds a clear path given the obstacle corners
        double FindClearPath(std::vector<std::vector<int>> interest_points,
                           shared_ptr<pcl::visualization::PCLVisualizer> viewer);

        //Determines whether the input path is obstructed
        bool CheckPath(std::vector<std::vector<int>> interest_points,
               shared_ptr<pcl::visualization::PCLVisualizer> viewer,
               std::vector<int> &obstacles, compareLine leftLine, compareLine rightLine);

    public:
        //Main function that runs the above 
        obstacle_return pcl_obstacle_detection(shared_ptr<pcl::visualization::PCLVisualizer> viewer);

        //Creates a point cloud visualizer
        shared_ptr<pcl::visualization::PCLVisualizer> createRGBVisualizer();

        //Cleares and resizes cloud for new data
        void update();
};

#endif