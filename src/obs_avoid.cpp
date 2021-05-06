
#include "obs_avoid.h"

void VeloObs::handleObstacle(const sensor_msgs::PointCloud2::ConstPtr& ros_pc){
	ROS_INFO("flag0");

	boost::recursive_mutex::scoped_lock cmd_lock(scope_mutex_);	
	std_msgs::Float32MultiArray dists;
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
	seg.setDistanceThreshold (0.01);          
	seg.segment (*inliers, *coefficients);    
	
	ROS_INFO("flag1");

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
	ROS_INFO("flag2");

	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);	    
	if(output_ptr->size() != 0){	
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		tree->setInputCloud (output_ptr);  
		std::vector<int> nn_indices(30);
		std::vector<float> nn_dists(30);
		pcl::PointXYZ origin(0, 0, 0);

		tree->nearestKSearch(origin, 30, nn_indices, nn_dists);

		float min_negative = -999, min_positive = 999, movement = 0, min_positive_point =0, min_negative_point =0;
		int count_negative=0, count_positive = 0;
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
		ROS_INFO("flag3");

            if(count_positive > count_negative) //obstacles in left side
            {
                // shift_position_ = std::min(params_.obstacle_coefficient_/output_ptr->points[nn_indices[min_positive_point]].x, 0.01);
                std::cout<<"[LEFT OBSTACLES] Distance to obstacles(m): "<<abs(output_ptr->points[nn_indices[min_positive_point]].x) <<std::endl;	
                dists.data[0] = abs(output_ptr->points[nn_indices[min_positive_point]].x);
                dists.data[1] = abs(output_ptr->points[nn_indices[min_positive_point]].y);
            }
            else	//obstacles in right side
            {	
                // shift_position_ = std::max(-params_.obstacle_coefficient_/output_ptr->points[nn_indices[min_negative_point]].x, -0.01);
                std::cout<<"[RIGHT OBSTACLES] Distance to obstacles(m): "<<abs(output_ptr->points[nn_indices[min_negative_point]].x) <<std::endl;
                dists.data[0] = abs(output_ptr->points[nn_indices[min_negative_point]].x) ;
                dists.data[1] = abs(output_ptr->points[nn_indices[min_negative_point]].x) ;
            }	
            
	}

	//else
		//std::cout<<"[Go straight] No obstacles were detected" << std::endl;
	
ROS_INFO("flag4");

// Convert data type PCL to ROS
	sensor_msgs::PointCloud2 ros_output;
	pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
	
	pcl_conversions::fromPCL(pcl_pc, ros_output);

	// Publish the data
	pub_obs_.publish(ros_output);
    pub_obs_dists_.publish(dists);
	}


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "velodyne_obstacle_node");
	VeloObs VeloObs;
	if (VeloObs.configure())
		while(ros::ok()) 
    	    ros::spinOnce();
  	return 0;
}
