#pragma once
#include <opencv2/opencv.hpp>
#include "perception.hpp"

#if OBSTACLE_DETECTION
	#include <pcl/common/common_headers.h>
#endif


class Camera {
private:
	class Impl;
	Impl *impl_;
	std::string rgb_foldername;
	std::string depth_foldername;
	std::string pcl_foldername;

	cv::VideoWriter vidWrite;
	
public:
	Camera();
	~Camera();

	bool grab();

	cv::Mat image();
	cv::Mat depth();
	

	#if OBSTACLE_DETECTION
	//void getDataCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud);

	template <typename PointInT>
	void getDataCloud(pcl::PointCloud<PointInT> &p_pcl_point_cloud);	


	// <typename PointInT> 
	//void getDataCloud(PointInT &p_pcl_point_cloud);
	#endif

	#if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
	void disk_record_init();
	//template <typename PointInT>
	//void write_curr_frame_to_disk(cv::Mat rgb, cv::Mat depth, PointInT &p_pcl_point_cloud, int counter);	
	void write_curr_frame_to_disk(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud, int counter);
	#endif
/*
	#if WRITE_CURR_FRAME_TO_DISK && AR_DETECTION && OBSTACLE_DETECTION
	void disk_record_init();
	void write_curr_frame_to_disk(cv::Mat rgb, cv::Mat depth, pcl::PointCloud<pcl::PointXYZRGB>::Ptr &p_pcl_point_cloud, int counter);
	#endif
*/
	void record_ar_init();
	void record_ar(cv::Mat rgb);
	void record_ar_finish();
};