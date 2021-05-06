
#ifndef OBS_AVOID_H
#define OBS_AVOID_H

#include "parameter.h"
#include <iostream>
#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>

#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_ros/transforms.h>
#include <pcl/filters/conditional_removal.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float32MultiArray.h>


using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

class VeloObs
{
public:
    VeloObs():pnh_("~") {
		sub_obs_ = nh_.subscribe("/velodyne_points", 10, &VeloObs::handleObstacle, this);
		pub_obs_ = nh_.advertise<sensor_msgs::PointCloud2> ("/cropped_obs", 10);
        pub_obs_dists_ = nh_.advertise<std_msgs::Float32MultiArray> ("/obs_dists", 10);
    };

    void handleObstacle(const sensor_msgs::PointCloud2::ConstPtr& ros_pc); 
	
	bool configure()
	{
        if (!params_.load(pnh_))
        {
            ROS_ERROR("Failed to load parameters (velo)");
            return false;
        }
		return true;
	}
    ~VeloObs(){}

private:
    ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
        
    ros::Subscriber sub_obs_;
    ros::Publisher pub_obs_;
	ros::Publisher pub_obs_dists_;
	boost::recursive_mutex scope_mutex_;
	VeloParameters params_;
    float shift_position_ = 0;      
};
#endif
