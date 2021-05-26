
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

    void handleObstacle(const sensor_msgs::PointCloud2::ConstPtr& ros_pc)
{

	std_msgs::Float32MultiArray dists;
	dists.data.clear();
	pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
	
	pcl_conversions::toPCL(*ros_pc, pcl_pc);
	// Convert point cloud to PCL native point cloud
	PointCloud::Ptr input_ptr(new PointCloud());
	pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

	// Create output point cloud
	PointCloud::Ptr output_ptr(new PointCloud());    	    

	// Object for storing the plane model coefficients.
	pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
	pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
								
	pcl::SACSegmentation<pcl::PointXYZ> seg;
	seg.setOptimizeCoefficients (true);      
	seg.setInputCloud (input_ptr);                 
	seg.setModelType (pcl::SACMODEL_PLANE);    
	seg.setMethodType (pcl::SAC_RANSAC);      
	seg.setMaxIterations (1000);              
	seg.setDistanceThreshold (0.1);          
	seg.segment (*inliers, *coefficients);    

	pcl::PassThrough<pcl::PointXYZ> pass;

	pass.setInputCloud (input_ptr);         
	pass.setFilterFieldName ("y");         
	pass.setFilterLimits (-0.3, 0.3);    
	pass.filter (*output_ptr);              
																																																							
	pass.setInputCloud(output_ptr);
	pass.setFilterFieldName("z");           
	pass.setFilterLimits(-0.1, 0.5);       
	pass.filter(*output_ptr);             

	pass.setInputCloud (output_ptr);      
	pass.setFilterFieldName ("x");        
	pass.setFilterLimits (0, 1);          
	pass.filter (*output_ptr);           

	if(output_ptr->size() != 0){
		pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		sor.setInputCloud (output_ptr);  
		sor.setMeanK (50);               
		sor.setStddevMulThresh (1.0);    
		sor.filter (*output_ptr);        
	}

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);	    
	if(output_ptr->size() != 0){	
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (output_ptr);  
		std::vector<int> nn_indices(30);
		std::vector<float> nn_dists(30);
		pcl::PointXYZ origin(0, 0, 0);

		tree->nearestKSearch(origin, 30, nn_indices, nn_dists);

		float min_negative = -999, min_positive = 999, movement = 0, min_positive_point =0, min_negative_point =0;
		int count_negative=0, count_positive = 0, count_center = 0;
		float point_x, point_y; 
		//front_obstacle_ = false;
		if(nn_indices.size()>5)
			for (int i = 0; i < nn_indices.size(); i++)
			{
				point_y = output_ptr->points[nn_indices[i]].y;
				point_x = output_ptr->points[nn_indices[i]].x;
				

				if(point_y > 0) //obstacles in left side
				{			
					if(min_positive >point_y)
					{
						min_positive = point_y;
						min_positive_point = i;
					}
					count_positive += 1;
				}
				else //obstacles in right side
				{
					if(min_negative < point_y)
					{	
						min_negative = point_y;
						min_negative_point = i;
					}
					count_negative += 1;
				}
			}

            if(count_positive > count_negative) //obstacles in left side
            {
                // shift_position_ = std::min(params_.obstacle_coefficient_/output_ptr->points[nn_indices[min_positive_point]].x, 0.01);
                //std::cout<<"[LEFT OBSTACLES] Distance to obstacles(m): "<<abs(output_ptr->points[nn_indices[min_positive_point]].x) <<std::endl;	
                dists.data.push_back(output_ptr->points[nn_indices[min_positive_point]].x);
                dists.data.push_back(output_ptr->points[nn_indices[min_positive_point]].y);
            }
            else	//obstacles in right side
            {	
                // shift_position_ = std::max(-params_.obstacle_coefficient_/output_ptr->points[nn_indices[min_negative_point]].x, -0.01);
                //std::cout<<"[RIGHT OBSTACLES] Distance to obstacles(m): "<<abs(output_ptr->points[nn_indices[min_negative_point]].x) <<std::endl;
                dists.data.push_back(output_ptr->points[nn_indices[min_negative_point]].x);
                dists.data.push_back(output_ptr->points[nn_indices[min_negative_point]].y);
            }	
            
	}

	//else
		//std::cout<<"[Go straight] No obstacles were detected" << std::endl;

// Convert data type PCL to ROS
	sensor_msgs::PointCloud2 ros_output;
	pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
	
	pcl_conversions::fromPCL(pcl_pc, ros_output);

	// Publish the data
	pub_obs_.publish(ros_output);
    pub_obs_dists_.publish(dists);
	}
	
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
