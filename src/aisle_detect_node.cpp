#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <tf/transform_listener.h>

namespace auto_driving {

class AisleDetectNode : public nodelet::Nodelet {

public:
	AisleDetectNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();
        
        // Configuration //
        nhp.param("line_thresh", config_.line_thresh_, 0.5);
        nhp.param("aisle_width", config_.aisle_width_, 0.6);

        // Subscriber & Publisher
        sub_scan_ = nhp.subscribe("/up_scan", 10, &AisleDetectNode::scanCallback, this);

        pub_line_ = nhp.advertise<sensor_msgs::PointCloud2>("aisle/cluster_line", 10);
        pub_points_ = nhp.advertise<sensor_msgs::PointCloud2> ("aisle/points_msg", 10);
        pub_y_err_local_ = nhp.advertise<std_msgs::Float32> ("aisle/y_local_err", 10);
	};

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
	{
        laser_geometry::LaserProjection projector_;
        tf::TransformListener tfListener_;
        ROS_INFO("scan callback");
        sensor_msgs::PointCloud2 cloud;
        projector_.transformLaserScanToPointCloud("base_link", *scan_msg, cloud, tfListener_);

        std_msgs::Float32 y_err_local;
        y_err_local.data = 0.1;
        pub_y_err_local_.publish(y_err_local);
	}

private:
	// Publisher & Subscriber
	ros::Subscriber sub_scan_;
	ros::Publisher pub_line_;
	ros::Publisher pub_points_;
	ros::Publisher pub_y_err_local_;

	/** configuration parameters */
	typedef struct
	{
		double line_thresh_;
		double aisle_width_;
	} Config;
	Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::AisleDetectNode, nodelet::Nodelet);

