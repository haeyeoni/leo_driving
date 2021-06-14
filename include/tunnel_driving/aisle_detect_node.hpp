#ifndef AISLE_DETECT_NODE_H
#define AISLE_DETECT_NODE_H

#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <laser_geometry/laser_geometry.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/conditional_removal.h>

#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>

#include <nodelet/nodelet.h>

class AisleDetectNode : public nodelet::Nodelet
{
public:
	virtual void onInit();

private:
	// Aisle detect model
	AisleDetect aisle_detector_;

	// Publisher & Subscriber
	ros::Subscriber sub_scan_;
	ros::Publisher pub_line_;
	ros::Publisher pub_points_;
	void set_config();
	void scanCallback(const sensor_msgs::LaserScanPtr & scan_msg);

	/** configuration parameters */
	typedef struct
	{
		double line_thresh_;
		double aisle_width_;
	} Config;
	Config config_;
}
#endif // AISLE_DETECT_NODE_H