
#include <laser_geometry/laser_geometry.h>
#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <std_msgs/Float32.h>
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
        sub_scan_ = nhp.subscribe("/rp/scan", 10, &AisleDetectNode::scanCallback, this);

        pub_line_ = nhp.advertise<sensor_msgs::PointCloud2>("/cluster_line", 10);
        pub_points_ = nhp.advertise<sensor_msgs::PointCloud2> ("/aisle_points", 10);
	};

	void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
	{
    	// 1. Data type conversions (laser scan -> pointcloud2)
	    laser_geometry::LaserProjection projector;
        sensor_msgs::PointCloud2 cloud_msg;
		projector.projectLaser(*scan_msg, cloud_msg);
		
		pcl::PCLPointCloud2::Ptr temp_cloud (new pcl::PCLPointCloud2);
        pcl_conversions::toPCL(cloud_msg, *temp_cloud); // save cloud message to cloud2
        pcl::PointCloud<pcl::PointXYZ>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZ>);
        pcl::fromPCLPointCloud2(*temp_cloud, *cloud);

		// 2. Crop Point Cloud
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inrange(new pcl::PointCloud<pcl::PointXYZ>); // <- cropped cloud
			// set condition
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -config_.aisle_width_)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, config_.aisle_width_)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.0)));
			// conditional removal
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setInputCloud(cloud);
		condrem.setCondition(range_condition);
		condrem.setKeepOrganized(true);
		condrem.filter(*cloud_inrange);
		if (cloud_inrange->size() == 0)
		{
			ROS_WARN("all points are cropped");
			return;
		}
		
		// 3. EXTRACT LINE (RANSAC ALGORITHM) 
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE); // <- extract model setting
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(config_.line_thresh_); // <- threshold (line width) // 0.5
		seg.setInputCloud(cloud_inrange); 
		seg.segment(*inliers, *coefficients);
		extract.setInputCloud(cloud_inrange);
		extract.setIndices(inliers);
		extract.setNegative(false); //<- if true, it returns point cloud except the line.
		extract.filter(*cloud_inrange);

		// 4. Extract Line Cluster
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cluster(new pcl::search::KdTree<pcl::PointXYZ>);
		tree_cluster->setInputCloud(cloud_inrange);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setInputCloud(cloud_inrange);
		ec.setClusterTolerance(0.05); // <- If the two points have distance bigger than this tolerance, then points go to different clusters. 
		ec.setMinClusterSize(30); 
		ec.setMaxClusterSize(800);;
		ec.setSearchMethod(tree_cluster);
		ec.extract(cluster_indices);		

			// extract first clustering (center cluster)
		int j = 0;
		std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			if (j == 0) {
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				{		
					cloud_cluster->points.push_back (cloud_inrange->points[*pit]);
				}
				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;
			}
			j++;
		}
		if((*cloud_cluster).size() == 0)
		{
			ROS_WARN("Not enough points!");
			return;
		} 
		
		// 5. FIND NEAREST POINT FROM THE ORIGIN ( => /nearest_point)
		pcl::PointCloud<pcl::PointXYZ> point_set; // Data to be published

		pcl::PointXYZ origin(0, 0, 0);	
		pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
		tree_->setInputCloud(cloud_cluster);
		std::vector<int> nn_indices(1); 
		std::vector<float> nn_dists(1);
		tree_->nearestKSearch(origin, 1, nn_indices, nn_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
		point_set.push_back(cloud_cluster->points[nn_indices[0]]); // [0]: closest

		// 6. Calculate the Reference point
		float sum_x = 0;
		int num_points = 0;

		// Update Line min & Line max
		float line_start_y = 0;
		float line_end_y = 1000;
		pcl::PointCloud<pcl::PointXYZ> line_cloud;
		
		for (int i = 0; i < (*cloud_cluster).size(); i++) 
		{
			sum_x += cloud_cluster->points[i].x;
			num_points ++;
			if (line_end_y > cloud_cluster->points[i].y) 
				line_end_y = cloud_cluster->points[i].y;
			if (line_start_y < cloud_cluster->points[i].y)
				line_start_y = cloud_cluster->points[i].y;
		}
		pcl::PointXYZ reference (sum_x / (float)num_points, (line_start_y + line_end_y)/2, 0);//Jinsuk				
		point_set.push_back(reference); // [1]: reference
		
		pcl::PointXYZ line_start_point (sum_x / (float)num_points, line_start_y, 0); 
		pcl::PointXYZ line_end_point (sum_x / (float)num_points, line_end_y, 0); 
		point_set.push_back(line_start_point); //[2]: line start point
		point_set.push_back(line_end_point); //[3]: line end point
		
		// Publish ROS Topics 
		sensor_msgs::PointCloud2 points_msg;
		sensor_msgs::PointCloud2 points_line;
		 
		pcl::toROSMsg((*cloud_cluster), points_line);
		pcl::toROSMsg(point_set, points_msg);
		points_line.header.frame_id = scan_msg->header.frame_id;
		points_msg.header.frame_id = scan_msg->header.frame_id;	
		this->pub_points_.publish(points_msg);
		this->pub_line_.publish(points_line);	
	}

private:
	// Publisher & Subscriber
	ros::Subscriber sub_scan_;
	ros::Publisher pub_line_;
	ros::Publisher pub_points_;

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

