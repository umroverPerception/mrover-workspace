#include "perception.hpp"
#include "rover_msgs/Target.hpp"
#include "rover_msgs/TargetList.hpp"
#include <unistd.h>
#include <deque>

using namespace cv;
using namespace std;
using namespace std::chrono_literals;
 
int main() {
  
  /* --- Camera Initializations --- */
  //Populate Constants from Config File
        MAX_FIELD_OF_VIEW_ANGLE{mRoverConfig["pt_cloud"]["max_field_of_view_angle"].GetInt()},
        PT_CLOUD_WIDTH{mRoverConfig["pt_cloud"]["pt_cloud_width"].GetInt()},
        PT_CLOUD_HEIGHT{mRoverConfig["pt_cloud"]["pt_cloud_height"].GetInt()},
        HALF_ROVER{mRoverConfig["pt_cloud"]["half_rover"].GetInt()},
        UP_BD_Z{mRoverConfig["pt_cloud"]["pass_through"]["upper_bd_z"].GetDouble()},
        UP_BD_Y{mRoverConfig["pt_cloud"]["pass_through"]["upper_bd_y"].GetDouble()},
        LOW_BD{mRoverConfig["pt_cloud"]["pass_through"]["lower_bd"].GetDouble()},
        ROVER_W_MM{mRoverConfig["pt_cloud"]["rover_w_mm"].GetDouble()},
        LEAF_SIZE{mRoverConfig["pt_cloud"]["downsample_voxel_filter"].GetFloat()},
        MAX_ITERATIONS{mRoverConfig["pt_cloud"]["ransac"]["max_iterations"].GetInt()},
        SEGMENTATION_EPSLION{mRoverConfig["pt_cloud"]["ransac"]["segmentation_epsilon"].GetDouble()},
        DISTANCE_THRESHOLD{mRoverConfig["pt_cloud"]["ransac"]["distance_threshold"].GetDouble()},
        CLUSTER_TOLERANCE{mRoverConfig["pt_cloud"]["euclidean_cluster"]["cluster_tolerance"].GetInt()},
        MIN_CLUSTER_SIZE{mRoverConfig["pt_cloud"]["euclidean_cluster"]["min_cluster_size"].GetInt()},
        MAX_CLUSTER_SIZE{mRoverConfig["pt_cloud"]["euclidean_cluster"]["max_cluster_size"].GetInt()},
        
        //Other Values
        bearing{0}, distance{0}, detected{false},
        pt_cloud_ptr{new pcl::PointCloud<pcl::PointXYZRGB>} {

        #if PERCEPTION_DEBUG
        viewer = createRGBVisualizer(); //This is a smart pointer so no need to worry ab deleteing it
        viewer_original = createRGBVisualizer();
        #endif

        #if ZED_SDK_PRESENT
           sl::Resolution cloud_res = sl::Resolution(PT_CLOUD_WIDTH, PT_CLOUD_HEIGHT);
           cloudArea = cloud_res.area();
        #else
            cloudArea = PT_CLOUD_WIDTH*PT_CLOUD_HEIGHT;
        #endif

  }


    /* --- Wrap Things Up --- */
    #if AR_RECORD
        cam.record_ar_finish();
    #endif
  
    return 0;
}

