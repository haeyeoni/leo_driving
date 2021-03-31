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
	seg.setDistanceThreshold(0.05); // <- threshold (line width) // 0.5
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
		//pcl::PointXYZ reference (sum_x / (float)num_points, sum_y / (float)num_points, 0);
		pcl::PointXYZ reference (sum_x / (float)num_points, (LINE_START + LINE_END)/2, 0);//Jinsuk		
				
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
	{
		cout<<"B push"<<endl;
		driving_mode_ += 1; 
	}
	
	if (joy_msg->buttons[0] == 1)	
	{
		cout<<"A push"<<endl;
		return_mode_ +=1;
	}
	if(return_mode_ %2 ==1)
	{
		fully_autonomous_ = !fully_autonomous_; 
	}
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
#if HY_CONTROL //1: Previous control , 0: Jinsuk control
	if(fully_autonomous_) // Automatically set goal alternatively 
	{// check which goal should be used 
		if (is_first_goal_)
			setGoal(first_goal_);			
		else
			setGoal(second_goal_);	
	}
#endif

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
	/*float y_err_local = shift_position_;
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
	} */
#if HY_CONTROL //1: Previous control , 0: Jinsuk control
// 2. CHECK GLOBAL GOAL
	has_arrived_ = checkArrival();
	//cout<<"has arrived"<<has_arrived_<<endl;
	
// 3. PUBLISH COMMAND
	/*if(y_err_local > params_.adjusting_y_err_bound_)
	{		
		cout<<"adjusting y err: "<< y_err_local <<endl;
		
		cmd_vel.linear.x = 0.0; //TODO from jinsuk
		cmd_vel.angular.z = -Kpy*y_err_local;
	}*/
	//if(y_err_local > params_.adjusting_y_err_bound_)
	if(adjusting_angle) // To adjust heading after finishing rotating for 5s.
	{
		adjusting_angle_cnt++;
		
		cout<<"adjusting y err: "<< y_err_local << ", cnt: " << adjusting_angle_cnt <<endl;

		cmd_vel.linear.x = 0.0; //TODO from jinsuk
		cmd_vel.angular.z = -Kpy*y_err_local;
		
		if(adjusting_angle_cnt>= 70 || y_err_local < params_.adjusting_y_err_bound_)
		{
			cout<<"adjusting end" <<endl;
			adjusting_angle_cnt=0;
			adjusting_angle = false;
		}
	}
	else if ((has_goal_ && has_arrived_) || front_obstacle_)
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;		
	}	
	else
	{
		//cmd_vel.linear.x = linear_vel; //TODO from jinsuk
		if(params_.Kpx_param_*dist_err_global > linear_vel)
			cmd_vel.linear.x = linear_vel;
		else
			cmd_vel.linear.x =params_.Kpx_param_*dist_err_global;
		
		cmd_vel.angular.z = -Kpy*y_err_local;
	}
	if(!is_rotating_)
		pub_cmd_.publish(cmd_vel);	
	printf("\n");

#else //Jinsuk control
	if(fully_autonomous_) // Automatically set goal alternatively 
	{// check which goal should be used 
		if (is_first_goal_)
			goal_point_.pose.position = first_goal_.pose.position;
		else
			goal_point_.pose.position = second_goal_.pose.position;
		cout<<"goal is set: "<<goal_point_.pose.position.x<<" "<<goal_point_.pose.position.y<<endl;		
	}

	double pinpoint_x, pinpoint_y, pinpoint_z ,pinpoint_theta;
	double angle_err;

	const geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
	tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
	tf2::Quaternion orientation (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);

	tf2::Matrix3x3 m(orientation);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);

	x_err_global = goal_point_.pose.position.x - translation.x();
	y_err_global = goal_point_.pose.position.y - translation.y();

	dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);
//	dist_err_global = sqrt(x_err_global*x_err_global);//Temporary because of AMCL's wrong data
	if (front_obstacle_)
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;		
		pub_cmd_.publish(cmd_vel);
		return;
	}
	if(read_pose_ && params_.amcl_mode_)
	{
		if (dist_err_global < params_.global_boundary_)
		{
			if(rotating_flag) //It's not using now
			{
				pinpoint_x = translation.x();
				pinpoint_y = translation.y();
				pinpoint_z = translation.z();
				pinpoint_theta = yaw; 
				transition_flag =1;
				rotating_flag =0;
			}
			//angle_err = goal_point_.pose.position.yaw - yaw; //TODO global yaw
			angle_err = pinpoint_theta - yaw; //TODO delete because it's for compiling
			if(angle_err > M_PI)
				angle_err -= 2*M_PI;
			else if(angle_err < -M_PI)
				angle_err += 2*M_PI;

			cout<<"rotating ..."<<endl;						
			cmd_vel.linear.x = 0.0;
			//cmd_vel.angular.z = -Kpy_param_rot_;
			cmd_vel.angular.z = -params_.Kpy_param_rot_*angle_err;
			if(angle_err<params_.global_boundary_ && fully_autonomous_)
			{
				is_first_goal_ = !is_first_goal_;
				adjusting_angle = true;
			}
		}
		else
		{
			if(transition_flag) 
			{
				transition_flag =0;
				rotating_flag=1;
			}
			//cmd_vel.linear.x = linear_vel; //TODO from jinsuk
			if(params_.Kpx_param_*dist_err_global > linear_vel)
				cmd_vel.linear.x = linear_vel;
			else
				cmd_vel.linear.x =params_.Kpx_param_*dist_err_global;
			
			cmd_vel.angular.z = -Kpy*y_err_local;
		}
		if(adjusting_angle) // To adjust heading after finishing rotating for 5s.
		{
			adjusting_angle_cnt++;
			
			cout<<"adjusting y err: "<< y_err_local << ", cnt: " << adjusting_angle_cnt <<endl;
			
			cmd_vel.linear.x = 0.0; //TODO from jinsuk
			cmd_vel.angular.z = -Kpy*y_err_local;
			
			if(adjusting_angle_cnt>= 70 || y_err_local < 0.01)
			{
				cout<<"adjusting end" <<endl;
				adjusting_angle_cnt=0;
				adjusting_angle = false;
			}
		}
		pub_cmd_.publish(cmd_vel);
	}
	else if(params_.gmapping_mode_)
	{
		if ((has_goal_ && has_arrived_) || front_obstacle_)
		{
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;		
		}	
		else
		{
			//cmd_vel.linear.x = linear_vel; //TODO from jinsuk
			cmd_vel.linear.x = linear_vel;
			cmd_vel.angular.z = -Kpy*y_err_local;
		}
		if(!is_rotating_)
			pub_cmd_.publish(cmd_vel);
	}
	
#endif
}

bool Command::checkArrival()
{
	if (read_pose_ && has_goal_) // Drive until the robot arives to the goal
	{
		boost::recursive_mutex::scoped_lock pose_lock(scope_mutex_);
		//float x_err_global, y_err_global, dist_err_global = 0.0; // global x, y, dist err
		const geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
		tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
		
	
		x_err_global = goal_point_.pose.position.x - translation.x();
		y_err_global = goal_point_.pose.position.y - translation.y();
//		dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);
		dist_err_global = sqrt(x_err_global*x_err_global);//Temporary

		if (dist_err_global < params_.global_boundary_)
		{
			cout<<"arrived to the goal: "<< dist_err_global <<endl;	
			if (fully_autonomous_)
			{
				is_first_goal_ = !is_first_goal_;
				//return false;			
			}	
			return true;
		}
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
	goal_point_.pose.position = click_msg->pose.position;
	cout<<"goal is set: "<<goal_point_.pose.position.x<<" "<<goal_point_.pose.position.y<<endl;		
		
	has_goal_ = 1;
	has_arrived_ = false;

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
				double pinpoint_theta = yaw;
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


void Command::setGoal(geometry_msgs::PoseStamped goal_pose)
{	
	goal_point_.pose.position = goal_pose.pose.position;
	cout<<"goal is set: "<<goal_point_.pose.position.x<<" "<<goal_point_.pose.position.y<<endl;		
		
	has_goal_ = 1;
	//has_arrived_ = false; //Jinsuk

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
		
			double angle = atan2(goal_pose.pose.position.y - translation.y(), goal_pose.pose.position.x - translation.x());
			cout<< "current (x,y): " <<translation.x() << ", " << translation.y() << endl;
			
			cout<<"angle between goal and current: " <<angle<<endl;
			cout<<"current angle: " <<yaw<<endl;
		
			//if (abs(angle - yaw) > params_.back_rotating_ang_)
			if (abs(angle - yaw) > params_.back_rotating_ang_ && has_arrived_== true)
			{
				double pinpoint_x = translation.x();
				double pinpoint_y = translation.y();
				double pinpoint_z = translation.z();
				double pinpoint_theta = yaw;
				rotateReverse(pinpoint_x, pinpoint_y, pinpoint_z, pinpoint_theta);
				adjusting_angle = true;
				has_arrived_ = false;
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
	float Kpy_rot = params_.Kpy_param_rot_; // rotation 
	float linear_vel_rot = params_.linear_vel_rot_;
	is_rotating_ = true; 
	double angle_err, dist_err, current_angle;
	while(ros::ok() && is_rotating_)
	{
		// if(this->driving_mode_ % 2 == 1)
			// break;
		const geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
		tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
		tf2::Quaternion orientation (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);

		tf2::Matrix3x3 m(orientation);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		cout<<"pinpoint_theta: "<<pinpoint_theta<<endl;
		cout<<"current yaw: "<<yaw<<endl;

		angle_err = pinpoint_theta - yaw;
		cout <<"first angle err: " << angle_err <<endl;
		
		if(angle_err > M_PI)
			angle_err = angle_err -2*M_PI;
		if(angle_err < -M_PI)
			angle_err = angle_err + 2*M_PI;


		dist_err = sqrt(pow(translation.x() - pinpoint_x, 2) + pow(translation.y() - pinpoint_y, 2));
		cout<<"angle error: "<<angle_err<<endl;
		cout<<"distance error: "<<dist_err<<endl;
		if(abs(angle_err) > M_PI - params_.rotation_ang_err_)
		{
			cout<<"finished rotating!"<<endl;
			is_rotating_ = false; //restart autonomous driving
			return;	
		}		
		else// if (dist_err < params_.rotation_dist_err_)
		{
			cout<<"rotating ..."<<endl;						
			cmd_vel.linear.x = 0.0;
			//cmd_vel.angular.z = -Kpy_rot;
			cmd_vel.angular.z = -Kpy_rot;
			pub_cmd_.publish(cmd_vel);
		}
#if 0
		else // moved too much from the pinpoint while rotation
		{
			cout<<"adjusting position"<<endl;
			double angle = atan2(translation.y() - pinpoint_y, translation.x() - pinpoint_x);
			cout<<"angle while adjusting pos: " << angle << endl;
			cmd_vel.angular.z = 0.0;
/*			
			if (abs(angle - yaw) > M_PI - params_.rotation_ang_err_) // should go front
				cmd_vel.linear.x = linear_vel_rot;
			else // should go back
				cmd_vel.linear.x = -linear_vel_rot;
*/
			if ((angle - pinpoint_theta) >= 0) // should go front
				cmd_vel.linear.x = linear_vel_rot;
			else // should go back
				cmd_vel.linear.x = -linear_vel_rot;
			pub_cmd_.publish(cmd_vel);
		}
#endif
		printf("\n");
		ros::Duration(0.5).sleep();
	}	
}

void Command::handleObstacle(const sensor_msgs::PointCloud2::ConstPtr& ros_pc)
	{
		boost::recursive_mutex::scoped_lock cmd_lock(scope_mutex_);
		
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
		
	    pcl::PassThrough<pcl::PointXYZ> pass;

	    pass.setInputCloud (input_ptr);         
	    pass.setFilterFieldName ("y");         
	    pass.setFilterLimits (-0.35, 0.35);    
	    pass.filter (*output_ptr);              
																																																								
	    pass.setInputCloud(output_ptr);
	    pass.setFilterFieldName("z");           
	    pass.setFilterLimits(-0.2, 0.5);       
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
	    if(output_ptr->size() != 0)
		{	
			//cout<<"check1"<<endl;
		    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		    tree->setInputCloud (output_ptr);  
		    std::vector<int> nn_indices(30);
		    std::vector<float> nn_dists(30);
		    pcl::PointXYZ origin(0, 0, 0);

		    tree->nearestKSearch(origin, 30, nn_indices, nn_dists);

			float min_negative = -999, min_positive = 999, movement = 0, min_positive_point =0, min_negative_point =0;
			int count_negative=0, count_positive = 0;
			
			float point_x, point_y, left_boundary, right_boundary, line_length;
			float line_start = LINE_START;
			float line_end = LINE_END;		
			line_length = line_end - line_start;
			front_obstacle_ = false;
			if(nn_indices.size()>5)
				for (int i = 0; i < nn_indices.size(); i++)
				{
								//cout<<"check2"<<endl;
					point_y = output_ptr->points[nn_indices[i]].y;
					point_x = output_ptr->points[nn_indices[i]].x;
					left_boundary = line_start + (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);
					right_boundary = line_end - (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);
					
					
					//if (abs(point_x) < params_.front_obstacle_dist_ && abs(left_boundary-point_y) < params_.robot_width_ && abs(right_boundary-point_y) < params_.robot_width_ )
					if (abs(point_x) < params_.front_obstacle_dist_ && abs(point_y) < params_.robot_width_/4)
					{
						cout<<"[FRONT OBSTACLES] Distance to obstacles(m): "<<point_x<<endl;
						front_obstacle_ = true;
						break;
					}
									
					if(point_y > 0) //obstacles in left side
					{			
			//cout<<"check3"<<endl;
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
				if(front_obstacle_ == false)
					if(count_positive > count_negative) //obstacles in left side
					{
						shift_position_ = +params_.obastalce_coefficient_/output_ptr->points[nn_indices[min_positive_point]].x;
						std::cout<<"[LEFT OBSTACLES] Distance to obstacles(m): "<<abs(output_ptr->points[nn_indices[min_positive_point]].x) <<std::endl;	
					}
					else	//obstacles in right side
					{	
						shift_position_ = -params_.obastalce_coefficient_/output_ptr->points[nn_indices[min_negative_point]].x;
						std::cout<<"[RIGHT OBSTACLES] Distance to obstacles(m): "<<abs(output_ptr->points[nn_indices[min_negative_point]].x) <<std::endl;
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
