// Copyright 2020, Postech, Cocel, Control Team

#include "cmd_vel_test.h"
#include "serial.hpp"

#define OpenMV_on 0

float LINE_START = 1;
float LINE_END = -1;


bool CHECK_LINE = false;
// OpemMV //
/*void *Openmv_control(void *data)
{
	std::shared_ptr<Serial> serial(new Serial(0, 115200));
//	Serial Openmv;
	serial->portOpen();
	unsigned char Openmv_data[100];
	unsigned int i=0;
	while(1)
	{
		serial->readBytes(&Openmv_data[i],1);
		cout<< Openmv_data[i];

		i++;
		
		if(i>=80)
		{
			i=0;
		}
	}	
}*/
/// CLASS 1 ///
void LineExtractRP::lineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in) 
{
	//boost::recursive_mutex::scoped_lock ransac_lock(scope_mutex_);
		
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

	// Update Line min & Line max
	LINE_START = 0;
	LINE_END = 1000;

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
	CHECK_LINE = true;

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

/// CLASS 2 ///
void Command::handleJoyMode(const sensor_msgs::Joy::ConstPtr& joy_msg){
	//Button "B" : driving mode change -->   even: auto, odd: joy control
	if (joy_msg->buttons[1] == 1)	
	{
		cout<<"B push"<<endl;
		joy_driving_ = !joy_driving_;
		ros::Duration(1).sleep();
	}
	
	if(joy_driving_)
	{ 
		geometry_msgs::Twist cmd_vel;
		cmd_vel.linear.x = joy_msg -> axes[1] * 0.5;
		cmd_vel.angular.z = joy_msg -> axes[0] * 0.5;
		pub_cmd_.publish(cmd_vel);
		return;
	}
}

void Command::publishCmd(const sensor_msgs::PointCloud2 &cloud_msg) 
{	
//	boost::recursive_mutex::scoped_lock cmd_lock(scope_mutex_);

	std_msgs::Bool arrival_flag, rotating_flag;
	arrival_flag.data = false;
	rotating_flag.data = false;

	if(joy_driving_) // joystick mode
		return;

	//else: autonomous driving
	geometry_msgs::Twist cmd_vel;
	float Kpy = params_.Kpy_param_; // rotation 
	float Kpy_int = params_.Kpy_int_param_;
	float linear_vel = params_.linear_vel_;
	
	PointCloud2Ptr tmp_cloud(new PointCloud2);
	pcl_conversions::toPCL(cloud_msg, *tmp_cloud);
	PointCloudPtr data_cloud(new PointCloud); 
	pcl::fromPCLPointCloud2(*tmp_cloud, *data_cloud); 

	//float secs = ros::Time::now().toSec();

	//cout<<"what time is it now??: " << secs << endl;

	if(!is_rotating_ && abs(obs_y_) < 0.4* params_.robot_width_){
		cout << "Front obstacle is detected" <<abs(obs_y_)<<endl;
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;
		pub_cmd_.publish(cmd_vel);
		return;

	}
// 1. CHECK THE FRONT OBSTACLES(go to 2.1)
	/*if (front_obstacle_)
	{
		cmd_vel.linear.x = 0.0;
		cmd_vel.angular.z = 0.0;		
		pub_cmd_.publish(cmd_vel);
		return;
	}*/

// 2. CALCULATE AISLE LINE LENGTH AND CENTER
	pcl::PointXYZ nearest_point = data_cloud->points[0];
	pcl::PointXYZ reference_point = data_cloud->points[1];
	
	//line_start>0(left),line_end<0(right)
	float line_start = LINE_START;
	float line_end = LINE_END;
	float line_length = line_end - line_start;
// 2.1 CHECK LINE VALIDATION
	/*if (line_length > params_.line_width_max_ || line_length < params_.line_width_min_)
	{
		ROS_WARN("currnent line length: %f \n line width is not proper. Go straight", line_length);
		// cmd_vel.linear.x = linear_vel * 0.5;
		// cmd_vel.angular.z = 0;
		// pub_cmd_.publish(cmd_vel);
		// return;	
	}*/
	float left_boundary = line_start - (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);
	float right_boundary = line_end + (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);

	float x_err_local = reference_point.x - nearest_point.x; 
	float y_err_local = reference_point.y - nearest_point.y;


	// Check boundary for safe driving (obs_x,obs_y)
	bool is_obs_in_aisle = obs_y_ > line_end && obs_y_ < line_start;
	if (is_obs_in_aisle && !is_rotating_) //modified
	{
		// 0. Front Obstacle check	
		/*if((obs_y_> 0 && line_end-obs_y_< 0.5 * params_.robot_width_) ||
		   (obs_y_ < 0 && obs_y_ - line_start < 0.5 * params_.robot_width_))			
		{
			cout << "line end = "<<line_end<<endl;
			cout << "line start = "<<line_start<<endl;
			cout << "obs_y_ = "<<obs_y_<<endl;
						
			cout << "Front obstacle is detected" <<endl;
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
			pub_cmd_.publish(cmd_vel);
			return;
		}*/
		
		// 0. Front Obstacle check
		
		/*if(obs_x_ < 0.5 && obs_x_>0 && abs(obs_y_)< 0.4* params_.robot_width_){
			cout << "Front obstacle is detected" <<endl;
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
			pub_cmd_.publish(cmd_vel);
			return;

		}	*/
		temp_is_obs_in_aisle = true;
		spare_length = 0;
		// 1. Right Obstacle Update	
		if(obs_y_ < 0 && obs_y_ > -1 && obs_x_ < 0.6)
		{	
			cout << "Right obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<endl;
			float shift = params_.obs_coefficient_*(line_end-obs_y_);
			y_err_local = (nearest_point.y + shift > left_boundary) ? left_boundary - nearest_point.y : y_err_local + shift;
	 	}
		// 2. Left Obstacle Update 
		else if(obs_y_ > 0 && obs_y_ < 1 && obs_x_ < 0.6)
		{
			cout << "Left obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<endl;
			float shift = params_.obs_coefficient_*(line_start-obs_y_);
			y_err_local = (nearest_point.y + shift < right_boundary) ? right_boundary - nearest_point.y : y_err_local + shift;
		}
	}

	// After obs disappear, go further 'spare_length'
	if(is_obs_in_aisle != temp_is_obs_in_aisle){ 
		spare_length += linear_vel * 0.1;
		y_err_local = 0;  
		cout<< "straight foward of spare distance" <<endl;
		if(spare_length > params_.spare_length_){
			spare_length = 0;
			temp_is_obs_in_aisle = false;
			cout<<"spare finish"<<endl;
		}
			
	}


// 3. GMAPPING MODE
	if (params_.mapping_mode_)
	{
		cmd_vel.linear.x = linear_vel;
		cmd_vel.angular.z = -Kpy*y_err_local;
		pub_cmd_.publish(cmd_vel);
		return;
	}

// 3. AMCL MODE
	else 
	{
	// 3.1 Set current goal 
		if (goal_count_ < params_.num_goals_)
		{
			// ROS_INFO("Waiting for inserting goal... ");
			return;
		}

		current_goal_ = goal_set_[goal_index_ % goal_count_];	
		/*tf2::Quaternion orientation (current_goal_.pose.orientation.x, current_goal_.pose.orientation.y, current_goal_.pose.orientation.z, current_goal_.pose.orientation.w);
		tf2::Matrix3x3 m(orientation);
		double goal_roll, goal_pitch, goal_yaw;
		m.getRPY(goal_roll, goal_pitch, goal_yaw);*/
		double goal_roll, goal_pitch;
		cout<<"goal is set: "<<current_goal_.pose.position.x<<" "<<current_goal_.pose.position.y<<endl;			

	// 3.2 Calculate Global Error
		x_err_global = current_goal_.pose.position.x - amcl_pose_.pose.pose.position.x;
		y_err_global = current_goal_.pose.position.y - amcl_pose_.pose.pose.position.y;
		double dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);		
		cout << "goal (x,y): " <<"(" <<current_goal_.pose.position.x << ", " <<current_goal_.pose.position.y << ")" <<endl;		
		cout << "curr (x,y): " <<"(" <<amcl_pose_.pose.pose.position.x << ", " << amcl_pose_.pose.pose.position.y << ")" <<endl;
		cout << "distance :" << dist_err_global <<endl;
		cout <<" " <<endl;
		

		if (dist_err_global < params_.global_dist_boundary_ || is_rotating_) // arrived to the goal position
		{
			yaw_err_integral = 0.0;
			const geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
			tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
			tf2::Quaternion orientation (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);
			tf2::Matrix3x3 m(orientation);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);

			if(!is_rotating_) 
			{
				cout<<"**Arrived to the goal position: "<<dist_err_global<<endl;
				arrival_flag.data = true;					
				goal_yaw = yaw + M_PI; // save current when start rotating
				if(goal_yaw > M_PI)
					goal_yaw -= 2*M_PI;
				else if(goal_yaw < -M_PI)
					goal_yaw += 2*M_PI;
				is_rotating_ = true;
			}
	// 3.3 Check whether robot should rotate
			


//To rotate 180 degree from the position where the mobile robot is arrived. // goalyaw mean "arrival yaw"


			double angle_err_global = goal_yaw - yaw;  
			cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<<endl;
			if(angle_err_global > M_PI)
				angle_err_global -= 2*M_PI;
			else if(angle_err_global < -M_PI)
				angle_err_global += 2*M_PI;

			cmd_vel.linear.x = 0.0;
			//cmd_vel.angular.z = -params_.Kpy_param_rot_*angle_err_global;
			double bounded_ang_err = min(abs(angle_err_global), 1.0);
			cmd_vel.angular.z = -params_.Kpy_param_rot_ * bounded_ang_err;
			cout<<"rotating ... bounded_angle_err: "<<cmd_vel.angular.z <<"angle: "<<angle_err_global<<endl;						
			
			if(abs(angle_err_global) < params_.global_angle_boundary_ && !adjusting_angle_)
			{
				cout<<"finish rotation"<<endl;
				rotating_flag.data = false; // finished rotation
				arrival_flag.data = false;
				goal_index_++;
				adjusting_angle_ = true; // finished rotation
			}
			
	// 3.4 If finish rotation, 
			if(adjusting_angle_)
			{
				adjusting_angle_count_++;		
				cout<<"adjusting y err: "<< y_err_local << ", cnt: " << adjusting_angle_count_ <<endl;
				cmd_vel.linear.x = 0.0;
						
				cmd_vel.angular.z = -Kpy*y_err_local;
				
				if(adjusting_angle_count_>= 70 || y_err_local < params_.adjusting_y_err_bound_)
				{
					cout<<"adjusting end" <<endl;
					adjusting_angle_count_=0;
					adjusting_angle_ = false;
					is_rotating_ = false;	
					rotating_flag.data = false;			
				}
			}
			pub_cmd_.publish(cmd_vel);
		}
		
		else // not arrived to the goal
		{
			if(params_.Kpx_param_*dist_err_global > linear_vel)
				cmd_vel.linear.x = linear_vel;
			else
				cmd_vel.linear.x = params_.Kpx_param_*dist_err_global;
//Previous version
			cmd_vel.angular.z = -Kpy*y_err_local; 
/*
//Normal PI
			yaw_err_integral = yaw_err_integral + y_err_local*0.1;
			cmd_vel.angular.z = -Kpy*y_err_local + Kpy_int*yaw_err_integral;

			
//Anti-wind up PI

			cmd_vel.angular.z = -Kpy*y_err_local + Kpy_int*yaw_err_integral;

			if(Outz_tmp > MAX_omega)
				Outz = MAX_omega;
			else if(Outz_tmp < -MAX_omega)
				Outz = -MAX_omega;
			else
				Outz = Outz_tmp;
			SatErr = Outz - Outz_tmp;
			yaw_err_integral = yaw_err_integral + (y_err_local + 1/Kpy*SatErr)*0.1;


			*/

		}
		pub_cmd_.publish(cmd_vel);	
		pub_arrival_.publish(arrival_flag);
		pub_rotating_.publish(rotating_flag); 
	}
}

void Command::handlePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
	amcl_pose_.pose.covariance = pose_msg->pose.covariance;
	amcl_pose_.pose.pose.position = pose_msg->pose.pose.position;
	amcl_pose_.pose.pose.orientation = pose_msg->pose.pose.orientation;
	read_pose_ = 1;
}   

void Command::handleObstacleDists(const std_msgs::Float32MultiArray::ConstPtr& dists_msg)
{

	if (dists_msg->data.size())
	{
		obs_x_ = dists_msg->data[0];
		obs_y_ = dists_msg->data[1];
	}
	else
	{
		obs_x_ = 1000000;
		obs_y_ = 1000000;
	}
}
	

void Command::setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
{	
	goal_count_ ++;
	ROS_INFO("%d th goal is set", goal_count_);
	goal_set_.push_back(*click_msg);
}   


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "leo_driving_node");
	LineExtractRP LineExtractRP;
	Command Command;	

/*
#if OpenMV_on
	pthread_t p_thread[1];
	int thr_id,a=1;
	thr_id = pthread_create(&p_thread[0], NULL, Openmv_control, (void *)&a);
	if (thr_id < 0)
	{
		perror("thread create error : ");
		exit(0);
	}
#endif
*/

	while(ros::ok()) 
	{
		if (LineExtractRP.params_.pseudo_rp_)
		{		
			pcl::PointXYZ dummy(0,0,0);   
			sensor_msgs::PointCloud2 points_msg;
			PointCloud point_set;
			point_set.push_back(dummy);
			point_set.push_back(dummy);
			pcl::toROSMsg(point_set, points_msg);
			points_msg.header.frame_id = "velodyne";	
			LineExtractRP.pub_points_.publish(points_msg);
			ros::Duration(0.1).sleep();
		}
		ros::spinOnce();
	}
	ros::shutdown();
  	return 0;
}
