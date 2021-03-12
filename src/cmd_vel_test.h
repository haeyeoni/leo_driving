//
// Created by haeyeon on 20. 6. 4..
//

#ifndef CMD_VEL_H
#define CMD_VEL_H

#include <iostream>
#include <string>
#include <boost/filesystem.hpp>
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
#include <mutex>
#include <cmath>

#include <ros/ros.h>
#include <ros/serialization.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

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

#include "parameter.h"

using namespace std;

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PCLPointCloud2 PointCloud2;
typedef PointCloud::Ptr PointCloudPtr;
typedef PointCloud2::Ptr PointCloud2Ptr;

class LineExtractRP
{
public:
    LineExtractRP() {
		sub_scan_ = nh_.subscribe("/up_scan", 10, &LineExtractRP::lineExtract, this);
		pub_line_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_line", 10);
		pub_nearest_ = nh_.advertise<sensor_msgs::PointCloud2> ("nearest_point", 10);
	 	pub_ref_ = nh_.advertise<sensor_msgs::PointCloud2> ("reference_point", 10);
		pub_points_ = nh_.advertise<sensor_msgs::PointCloud2> ("points_msg", 10);

    };

    void lineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in); 
	
    ~LineExtractRP(){}

    private:
        ros::NodeHandle nh_;
	ros::Subscriber sub_scan_;
        ros::Publisher pub_nearest_;
        ros::Publisher pub_ref_;
        ros::Publisher pub_points_;
	ros::Publisher pub_line_;
        laser_geometry::LaserProjection projector_;
		boost::recursive_mutex scope_mutex_;
};


class Command
{
public:
	Command():pnh_("~"), tfl_(tfbuf_) 
	{
		sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 5, &Command::handleJoyMode, this);
		sub_points_ = nh_.subscribe("/points_msg", 10, &Command::publishCmd,  this);
		sub_amcl_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &Command::handlePose, this);
		sub_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &Command::setGoal, this);
		// sub_obs_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &Command::handleObstacle, this);
		pub_cmd_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
		pub_obs_ = nh_.advertise<sensor_msgs::PointCloud2> ("/obstacles", 10);
	};

	bool configure()
	{
        if (!params_.load(pnh_))
        {
            ROS_ERROR("Failed to load parameters");
            return false;
        }
		return true;
	}

    	void publishCmd(const sensor_msgs::PointCloud2 &cloud_msg);	
	void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg);
	void handleObstacle(const sensor_msgs::PointCloud2::ConstPtr& ros_pc);

	void rotateReverse(double pinpoint_x, double pinpoint_y, double pinpoint_z, double pinpoint_theta);
	void handleJoyMode(const sensor_msgs::Joy::ConstPtr& joy_msg);
	bool checkArrival();
	void handlePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);

	~Command(){}

private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Subscriber sub_points_;
	ros::Subscriber sub_amcl_;
	ros::Subscriber sub_joy_;
	ros::Subscriber sub_goal_;
	ros::Subscriber sub_obs_;
	ros::Publisher pub_cmd_;
	ros::Publisher pub_obs_;
	geometry_msgs::PoseStamped clicked_point_;
	geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
	Parameters params_; 

	boost::recursive_mutex scope_mutex_;
	int driving_mode_ = 0; // even: auto, odd: joy control
	bool read_pose_ = false;
	bool has_goal_ = false;
	bool has_arrived_ = false;     
	float shift_position_ = 0;
	bool front_obstacle_ = false;	
	bool is_rotating_ = false;	
    
    // TF 
    tf2_ros::Buffer tfbuf_;
    tf2_ros::TransformListener tfl_;
    tf2_ros::TransformBroadcaster tfb_;
};

#endif 
