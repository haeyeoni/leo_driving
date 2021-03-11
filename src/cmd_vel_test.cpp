// Copyright 2020, Postech, Cocel, Control Team

#include "cmd_vel_test.h"


float LINE_START, LINE_END;
bool CHECK_LINE = false;
/// CLASS 1 ///
void LineExtractRP::lineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in) 
{
	boost::recursive_mutex::scoped_lock ransac_lock(scope_mutex_);
		
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

	if (!cloud_inrange->size())
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
	seg.setDistanceThreshold(0.5); // <- threshold (line width)
	seg.setInputCloud(cloud_inrange); 
	seg.segment(*inliers, *coefficients);
	extract.setInputCloud(cloud_inrange);
	extract.setIndices(inliers);
	extract.setNegative(false); //<- if true, it returns point cloud except the line.

	extract.filter(*cloud_inrange);
	
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
	if((*cloud_cluster).size()>0)
	{
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

		// Update Line min & Line max
		LINE_START = 1000;
		LINE_END = 0;

		for (int i = 0; i < (*cloud_cluster).size(); i++) 
		{
			filtered_cloud.push_back(cloud_cluster->points[i]);
			sum_x += cloud_cluster->points[i].x;
			sum_y += cloud_cluster->points[i].y;
			num_points ++;
			if (LINE_START > cloud_cluster->points[i].y) 
				LINE_START = cloud_cluster->points[i].y;
			if (LINE_END < cloud_cluster->points[i].y)
				LINE_END = cloud_cluster->points[i].y;
		}
		CHECK_LINE = true;

		PointCloud reference_cloud;
		pcl::PointXYZ reference (sum_x / (float)num_points, sum_y / (float)num_points, 0);	
				
		reference_cloud.push_back(reference);
		point_set.push_back(reference);

	// FIND LINE END AND LINE START
		pcl::PointXYZ left_infinite(0, -10000, 0);	
		pcl::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::KdTreeFLANN<pcl::PointXYZ>);
		tree_2->setInputCloud(cloud_cluster);
		std::vector<int> line_indices((*cloud_cluster).size()); 
		std::vector<float> line_dists((*cloud_cluster).size());
		tree_2->nearestKSearch(left_infinite, (*cloud_cluster).size(), line_indices, line_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
		
		point_set.push_back(cloud_cluster->points[line_indices[0]]);
		point_set.push_back(cloud_cluster->points[line_indices[-1]]);
		
	// PUBLISH ROS MESSAGES
		pcl::toROSMsg(closest, nearest_point);
		pcl::toROSMsg((*cloud_cluster), points_line);
		pcl::toROSMsg(reference_cloud, reference_point);
		pcl::toROSMsg(point_set, points_msg);
			
		reference_point.header.frame_id = scan_in->header.frame_id;
		nearest_point.header.frame_id = scan_in->header.frame_id;
		points_line.header.frame_id = scan_in->header.frame_id;
		points_msg.header.frame_id = scan_in->header.frame_id;
			
		this->pub_nearest_.publish(nearest_point);// current position		
		this->pub_ref_.publish(reference_point);
		this->pub_points_.publish(points_msg);
		this->pub_line_.publish(points_line);
	}
}

/// CLASS 2 ///
void Command::handleJoyMode(const sensor_msgs::Joy::ConstPtr& joy_msg){
	//Button "B" : driving mode change -->   even: auto, odd: joy control
	if (joy_msg->buttons[1] == 1)
		driving_mode_ += 1; 

	if(this->driving_mode_ % 2 == 1){ // odd: joy control
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = joy_msg -> axes[1]*0.5;
		cmd_vel.angular.z = joy_msg -> axes[0]*0.5;
		this->pub_cmd_.publish(cmd_vel);
	}
}

void Command::publishCmd(const sensor_msgs::PointCloud2 &cloud_msg) 
{	
	boost::recursive_mutex::scoped_lock cmd_lock(scope_mutex_);
		
	if(this->driving_mode_ % 2 == 1) // joystick mode
		return;

	//else: autonomous driving
	geometry_msgs::Twist cmd_vel;
	float Kpy = params_.Kpy_param_; // rotation 
	float linear_vel = params_.linear_vel_;
	
	PointCloud2Ptr tmp_cloud(new PointCloud2);
	pcl_conversions::toPCL(cloud_msg, *tmp_cloud);
	PointCloudPtr data_cloud(new PointCloud); 
	pcl::fromPCLPointCloud2(*tmp_cloud, *data_cloud); 
	
// 1. CALCULATE AISLE LINE LENGTH AND CENTER
	pcl::PointXYZ nearest_point = data_cloud->points[0];
	pcl::PointXYZ reference_point = data_cloud->points[1];
	if (!CHECK_LINE)
	{
		ROS_WARN("line was not detected yet");
		return;
	}
	float line_start = LINE_START;
	float line_end = LINE_END;
	float line_length = line_end - line_start;
	float left_boundary = line_start + (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);
	float right_boundary = line_end - (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);

	float x_err_local = reference_point.x - nearest_point.x; 
	float y_err_local = reference_point.y - nearest_point.y;

	// Check boundary for safe driving
	if(left_boundary < line_start + 0.5*line_length && right_boundary > line_end - 0.5*line_length)
	{			
		if (nearest_point.y + shift_position_ > left_boundary && nearest_point.y + shift_position_ < right_boundary)   			
		{
			y_err_local += shift_position_;
			// cout<<"in boundary"<<endl;
		}
		else if (nearest_point.y + shift_position_ < left_boundary && shift_position_ < 0)
		{
			y_err_local = left_boundary - nearest_point.y;
			// cout<<"out boundary, stay in left boundary"<<endl;
		}
		else if (nearest_point.y + shift_position_ > right_boundary && shift_position_ > 0)
		{
			y_err_local = right_boundary - nearest_point.y;
			// cout<<"out boundary, stay in right boundary"<<endl;
		}
	} 
// 2. CHECK GLOBAL GOAL
	bool has_arrived = checkArrival();
	float add_vel_local = 0;	

// 3. PUBLISH COMMAND
	if (has_arrived || front_obstacle_)
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;		
	}	
	else
	{
		cmd_vel.linear.x = linear_vel;
		cmd_vel.angular.z = -Kpy*y_err_local;
	}
	if(!is_rotating_)
		pub_cmd_.publish(cmd_vel);	
}

bool Command::checkArrival()
{
	if (read_pose_ && has_goal_) // Drive until the robot arives to the goal
	{
		boost::recursive_mutex::scoped_lock pose_lock(scope_mutex_);
		float x_err_global, y_err_global, dist_err_global = 0.0; // global x, y, dist err
		x_err_global = clicked_point_.pose.position.x - amcl_pose_.pose.pose.position.x;
		y_err_global = clicked_point_.pose.position.y - amcl_pose_.pose.pose.position.y;
		dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);
		return dist_err_global < 1.0;
	}

	return false;
}

void Command::handlePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
	amcl_pose_.pose.covariance = pose_msg->pose.covariance;
	amcl_pose_.pose.pose.position = pose_msg->pose.pose.position;
	amcl_pose_.pose.pose.orientation = pose_msg->pose.pose.orientation;
	read_pose_ = 1;
}   

void Command::setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
{	
	cout<<"goal is set"<<endl;
			
	clicked_point_.pose.position = click_msg->pose.position;
	has_goal_ = 1;

	// check the rotation orientation
	if (read_pose_)
	{
		try
		{
			const geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
			tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
			tf2::Quaternion orientation (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);

			tf2::Matrix3x3 m(orientation);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
		
			double angle = atan2(click_msg->pose.position.y - translation.y(), click_msg->pose.position.x - translation.x());
			
			cout<<"angle between goal and current: " <<angle<<endl;
			cout<<"current angle: " <<yaw<<endl;
		
			if (abs(angle - yaw) > params_.back_rotating_ang_)
			{
				double pinpoint_x = translation.x();
				double pinpoint_y = translation.y();
				double pinpoint_z = translation.z();
				double pinpoint_theta = yaw + M_PI;
				rotateReverse(pinpoint_x, pinpoint_y, pinpoint_z, pinpoint_theta);
			}
		}
		catch (tf2::TransformException& e)	
		{
			ROS_INFO("Failed to find transform between odom and base_link");
			return;		
		}
	}
}   


void Command::rotateReverse(double pinpoint_x, double pinpoint_y, double pinpoint_z, double pinpoint_theta)
{
	geometry_msgs::Twist cmd_vel;
	float Kpy = params_.Kpy_param_rot_; // rotation 
	float linear_vel_rot = params_.linear_vel_rot_;
	is_rotating_ = true; 
	double angle_err, dist_err, current_angle;
	while(true)
	{
		// if(this->driving_mode_ % 2 == 1)
			// break;
		const geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
		tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
		tf2::Quaternion orientation (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);

		tf2::Matrix3x3 m(orientation);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		
		cout<<"pinpoint_theta "<<pinpoint_theta<<endl;
		cout<<"current yaw"<<yaw<<endl;

		angle_err = pinpoint_theta - yaw;
		dist_err = sqrt(pow(translation.x() - pinpoint_x, 2) + pow(translation.y() - pinpoint_y, 2));
		
		if(angle_err > M_PI)
		{
			angle_err -= 2 * M_PI;		
		}
		else if(angle_err < -M_PI)
		{
			angle_err += 2 * M_PI;		
		}
		
		cout<<"angle error "<<angle_err<<endl;
		cout<<"distnace error"<<dist_err<<endl;

		if (abs(angle_err) < params_.rotation_ang_err_)
		{
			cout<<"finished rotating!"<<endl;
			break;
		}		
		else if (dist_err < params_.rotation_dist_err_)
		{
			cout<<"rotating ..."<<endl;						
			//cmd_vel.linear.x = 0.0;
			//cmd_vel.angular.z = -Kpy*angle_err;
			//pub_cmd_.publish(cmd_vel);
		}
		else // moved too much from the pinpoint while rotation
		{
			cout<<"adjusting position"<<endl;
			
			/*double angle = atan2(translation.y() - pinpoint_y, translation.x() - pinpoint_x);
			cmd_vel.angular.z = 0.0;
			if (abs(angle - yaw) > M_PI - params_.rotation_ang_err_) // should go front
				cmd_vel.linear.x = linear_vel_rot;
			else // should go back
				cmd_vel.linear.x = -linear_vel_rot;
			pub_cmd_.publish(cmd_vel);*/
		}
	}

	is_rotating_ = false; //restart autonomous driving
	
}

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "leo_driving_node");
	LineExtractRP LineExtractRP;
	Command Command;	
	if (Command.configure())
		while(ros::ok()) 
    	    ros::spinOnce();
  	return 0;
}
