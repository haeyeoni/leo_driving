//
// Created by haeyeon on 20. 6. 4..
//

#ifndef CMD_VEL_H
#define CMD_VEL_H

#include <iostream>
#include <string>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>

#include <stdint.h>
#include <unistd.h>
#include <termios.h>
#include <time.h>
#include <signal.h>
#include <string.h>
#include <memory>
#include <thread>
#include <chrono>
#include <cmath>

#include <ros/ros.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/Joy.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <laser_geometry/laser_geometry.h>

#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
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

#include "parameter.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

class LineExtractRP
{
public:
    LineExtractRP():pnh_("~") {
		if (!params_.load(pnh_))
        {
            ROS_ERROR("Failed to load parameters (ransac)");
        }

		sub_scan_ = nh_.subscribe("/up_scan", 10, &LineExtractRP::lineExtract, this);
		pub_line_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_line", 10);
		pub_nearest_ = nh_.advertise<sensor_msgs::PointCloud2> ("nearest_point", 10);
	 	pub_ref_ = nh_.advertise<sensor_msgs::PointCloud2> ("reference_point", 10);
		pub_points_ = nh_.advertise<sensor_msgs::PointCloud2> ("points_msg", 10);
    };

    void lineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in)
	{
		ROS_INFO("flag1");
		float Width = 0.6; //<- Data to be cropped (aisle width)
		// Messages to be published
		sensor_msgs::PointCloud2 nearest_point;
		sensor_msgs::PointCloud2 reference_point;      
		sensor_msgs::PointCloud2 points_msg;
		sensor_msgs::PointCloud2 points_line;
			
		// Message data before converted to the ROS messages
		PointCloud closest;
		PointCloud filtered_cloud;
		PointCloud point_set;
		PointCloud cluseter_line;

		sensor_msgs::PointCloud2 cloud_temp; // <- temporary point cloud to temporaly save the input point cloud     
		PointCloudPtr cloud(new PointCloud); // <- data cloud
		PointCloud2Ptr cloud2(new PointCloud2); // <- data cloud2 (converted from the cloud)
		PointCloudPtr cloud_inrange(new PointCloud); // <- cropped cloud
		
		// DATA TYPE CONVERSIONS: LaserScan (scan_in) -> PointCloud2 (cloud2)
		projector_.projectLaser(*scan_in, cloud_temp);
		pcl_conversions::toPCL(cloud_temp, *cloud2); 
		pcl::fromPCLPointCloud2(*cloud2, *cloud); 
		// CROP POINTCLOUD (cloud -> cloud_inrange) 
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
			// set condition
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -Width)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, Width)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.0)));
			// conditional removal
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setInputCloud(cloud);
		condrem.setCondition(range_condition);
		condrem.setKeepOrganized(true);
		condrem.filter(*cloud_inrange);

		ROS_INFO("flag2");
		if (cloud_inrange->size() == 0)
		{
			ROS_WARN("all points are cropped");
			return;
		}
		// EXTRACT LINE (RANSAC ALGORITHM): cloud_inragne changed 
		pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE); // <- extract model setting
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(params_.line_thresh_); // <- threshold (line width) // 0.5
		seg.setInputCloud(cloud_inrange); 
		seg.segment(*inliers, *coefficients);
		extract.setInputCloud(cloud_inrange);
		extract.setIndices(inliers);
		extract.setNegative(false); //<- if true, it returns point cloud except the line.

		extract.filter(*cloud_inrange);
		if (cloud_inrange->size() == 0)
		{
			ROS_WARN("all points are cropped");
			return;
		}
		// CENTER LINE CLUSTER IS EXTRACTED AMONG MULTIPLE LINE CLUSTERS 
			// clustering...

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

		ROS_INFO("flag3");
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
		// FIND NEAREST POINT FROM THE ORIGIN ( => /nearest_point)
		if((*cloud_cluster).size() == 0)
		{
			ROS_WARN("Not enough points!");
			return;
		} 

		pcl::PointXYZ origin(0, 0, 0);	
		pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
		tree_->setInputCloud(cloud_cluster);
		std::vector<int> nn_indices(1); 
		std::vector<float> nn_dists(1);
		tree_->nearestKSearch(origin, 1, nn_indices, nn_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
		
		closest.push_back(cloud_cluster->points[nn_indices[0]]); 
		point_set.push_back(closest[0]);

		// CALCULATE THE AVERAGE COORDINATES FROM THE CENTER LINE( => /reference_point)
		float current_x = cloud_cluster->points[nn_indices[0]].x;
		float current_y = cloud_cluster->points[nn_indices[0]].y;
		float threshold = 0.5;
		float threshold_y = 0.5;
		float sum_x = 0;
		float sum_y = 0;
		int num_points = 0;

		ROS_INFO("flag4");
		// Update Line min & Line max
		float LINE_START = 0;
		float LINE_END = 1000;

		for (int i = 0; i < (*cloud_cluster).size(); i++) 
		{
			filtered_cloud.push_back(cloud_cluster->points[i]);
			sum_x += cloud_cluster->points[i].x;
			sum_y += cloud_cluster->points[i].y;
			num_points ++;
			if (LINE_END > cloud_cluster->points[i].y) 
				LINE_END = cloud_cluster->points[i].y;
			if (LINE_START < cloud_cluster->points[i].y)
				LINE_START = cloud_cluster->points[i].y;
		}
		
		PointCloud reference_cloud;
		//pcl::PointXYZ reference (sum_x / (float)num_points, sum_y / (float)num_points, 0);
		pcl::PointXYZ reference (sum_x / (float)num_points, (LINE_START + LINE_END)/2, 0);//Jinsuk		
				
		reference_cloud.push_back(reference);
		point_set.push_back(reference);

		// FIND LINE END AND LINE START
		if((*cloud_cluster).size() == 0)
		{
			ROS_WARN("Not enough points!");
			return;
		} 
		pcl::PointXYZ left_infinite(0, -10000, 0);	
		pcl::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::KdTreeFLANN<pcl::PointXYZ>);
		tree_2->setInputCloud(cloud_cluster);
		std::vector<int> line_indices((*cloud_cluster).size()); 
		std::vector<float> line_dists((*cloud_cluster).size());
		tree_2->nearestKSearch(left_infinite, (*cloud_cluster).size(), line_indices, line_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
		
		pcl::PointXYZ line_start(0, LINE_START, 0);
		pcl::PointXYZ line_end(0, LINE_END, 0);
		
		point_set.push_back(line_start);
		point_set.push_back(line_end);
		
		// PUBLISH ROS MESSAGES
		pcl::toROSMsg(closest, nearest_point);
		pcl::toROSMsg((*cloud_cluster), points_line);
		pcl::toROSMsg(reference_cloud, reference_point);
		pcl::toROSMsg(point_set, points_msg);
			
		reference_point.header.frame_id = scan_in->header.frame_id;
		nearest_point.header.frame_id = scan_in->header.frame_id;
		points_line.header.frame_id = scan_in->header.frame_id;
		points_msg.header.frame_id = scan_in->header.frame_id;
			
		ROS_INFO("flag5");
		this->pub_nearest_.publish(nearest_point);// current position		
		this->pub_ref_.publish(reference_point);
		this->pub_points_.publish(points_msg);
		this->pub_line_.publish(points_line);	
	}

    ~LineExtractRP()
	{
		ros::shutdown();
	}

    private:
        ros::NodeHandle nh_;
		ros::NodeHandle pnh_;
		ros::Subscriber sub_scan_;
        ros::Publisher pub_nearest_;
        ros::Publisher pub_ref_;
        ros::Publisher pub_line_;
        laser_geometry::LaserProjection projector_;
		RansacParameters params_;
		ros::Publisher pub_points_;
		
};

class Command
{
public:
	Command():pnh_("~"), tfl_(tfbuf_) 
	{
		if (!params_.load(pnh_))
        {
            ROS_ERROR("Failed to load parameters (cmd)");
        }
	
		Kpy_ = params_.Kpy_param_; // rotation 
		linear_vel_ = params_.linear_vel_;
		
		sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("/joystick", 10, &Command::handleJoyMode, this);
		sub_points_ = nh_.subscribe("/points_msg", 10, &Command::updateLocalError,  this);
		sub_amcl_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &Command::amclDriving, this);
		sub_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &Command::setGoal, this);
		pub_cmd_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);

		pub_arrival_ = nh_.advertise<std_msgs::Bool> ("navigation/arrival", 10);
		pub_auto_mode_ = nh_.advertise<std_msgs::Bool> ("self_driving/auto_mode", 10);	
	
	};

    void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg);
	void handleJoyMode(const sensor_msgs::Joy::ConstPtr& joy_msg);
	void updateLocalError(const sensor_msgs::PointCloud2 &cloud_msg);	
	void amclDriving(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);
	
	~Command()
	{
		ros::shutdown();
	}

private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Subscriber sub_points_;
	ros::Subscriber sub_amcl_;
	ros::Subscriber sub_joy_;
	ros::Subscriber sub_goal_;
	ros::Publisher pub_cmd_;

	ros::Publisher pub_arrival_;
	ros::Publisher pub_auto_mode_;

	CmdParameters params_; 

	float Kpy_, linear_vel_;
	float y_err_local_ = 0;

	bool joy_driving_ = false; // even: auto, odd: joy control
	
	bool is_rotating_ = false;
	
    // TF 
    tf2_ros::Buffer tfbuf_;
    tf2_ros::TransformListener tfl_;
    tf2_ros::TransformBroadcaster tfb_;

	// GOAL
	int goal_index_ = 0;
	int goal_count_ = 0;
	std::vector<geometry_msgs::PoseStamped> goal_set_;
	geometry_msgs::PoseStamped current_goal_;
};

#endif 
