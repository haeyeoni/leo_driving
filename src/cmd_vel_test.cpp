// Copyright 2020, Postech, Cocel, Control Team

#include "cmd_vel_test.h"

void Command::setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
{	
	goal_count_ ++;
	ROS_INFO("%d th goal is set", goal_count_);
	goal_set_.push_back(*click_msg);
}   

void Command::handleJoyMode(const sensor_msgs::Joy::ConstPtr& joy_msg){
	//Button "B" : driving mode change -->   even: auto, odd: joy control
	if (joy_msg->buttons[1] == 1)	
	{
		cout<<"B push"<<endl;
		joy_driving_ = !joy_driving_;
		ros::Duration(1).sleep();
	}
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

void Command::updateLocalError(const sensor_msgs::PointCloud2 &cloud_msg) 
{	
	//Calculates local y error and update global variable (y_err_local_)
	PointCloud2Ptr tmp_cloud(new PointCloud2);
	pcl_conversions::toPCL(cloud_msg, *tmp_cloud);
	PointCloudPtr data_cloud(new PointCloud); 
	pcl::fromPCLPointCloud2(*tmp_cloud, *data_cloud); 

	pcl::PointXYZ nearest_point = data_cloud->points[0];
	pcl::PointXYZ reference_point = data_cloud->points[1];
	pcl::PointXYZ line_start = data_cloud->points[1];
	pcl::PointXYZ line_end = data_cloud->points[1];

	ref_y_ = reference_point.y;
	near_y_ = nearest_point.y;
	y_err_local_ = reference_point.y - nearest_point.y;
	start_y_ = line_start.y;
	end_y_ = line_end.y;
}

void Command::amclDriving(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
	if(joy_driving_) // joystick mode
		return;

	if (goal_count_ < 2)
	{
		ROS_INFO_ONCE("Waiting for inserting goal... ");
		return;
	}
	std_msgs::Bool auto_flag;
	auto_flag.data = !is_rotating_;
	current_goal_ = goal_set_[goal_index_ % goal_count_];	
	cout<<"goal is set: "<<current_goal_.pose.position.x<<" "<<current_goal_.pose.position.y<<endl;	
	

	// 3.2 Calculate Global Error
	float x_err_global = current_goal_.pose.position.x - pose_msg->pose.pose.position.x;
	float y_err_global = current_goal_.pose.position.y - pose_msg->pose.pose.position.y;
	double dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);	

	cout << "goal (x,y): " <<"(" <<current_goal_.pose.position.x << ", " <<current_goal_.pose.position.y << ")" <<endl;		
	cout << "curr (x,y): " <<"(" <<pose_msg->pose.pose.position.x << ", " << pose_msg->pose.pose.position.y << ")" <<endl;
	cout << "distance :" << dist_err_global <<endl;
	cout <<" " <<endl;
	

	double goal_yaw;	
	geometry_msgs::Twist cmd_vel;
	// 3.2.1 Not Arrived to the goal position
	if (dist_err_global > params_.global_dist_boundary_ && !is_rotating_) 
	{
		// TODO : ADD CONDITION
		/// IF (RP DRIVING)
		float y_err_local = y_err_local_;
		if(params_.check_obstacles_)
		{
			float line_length = end_y_ - start_y_;
			float left_boundary = start_y_ - (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);
			float right_boundary = end_y_ + (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);

			bool is_obs_in_aisle = obs_y_ > end_y_ && obs_y_ < start_y_;
			if (is_obs_in_aisle)
			{
				temp_is_obs_in_aisle = true;
				spare_length = 0;
				// 1. Right Obstacle Update	
				if(obs_y_ < 0 && obs_y_ > -1 && obs_x_ < 0.6)
				{	
					cout << "Right obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<endl;
					float shift = params_.obs_coefficient_*(end_y_-obs_y_);
					y_err_local = (near_y_ + shift > left_boundary) ? left_boundary - near_y_ : y_err_local_ + shift;
				}
				// 2. Left Obstacle Update 
				else if(obs_y_ > 0 && obs_y_ < 1 && obs_x_ < 0.6)
				{
					cout << "Left obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<endl;
					float shift = params_.obs_coefficient_*(start_y_-obs_y_);
					y_err_local = (near_y_ + shift < right_boundary) ? right_boundary - near_y_ : y_err_local + shift;
				}
			}
			if(is_obs_in_aisle != temp_is_obs_in_aisle)
			{ 
				spare_length += linear_vel_ * 0.1;
				y_err_local = 0;  
				cout<< "straight foward of spare distance" <<endl;
				if(spare_length > params_.spare_length_){
					spare_length = 0;
					temp_is_obs_in_aisle = false;
					cout<<"spare finish"<<endl;
				}	
			}
		}
		if(params_.Kpx_param_*dist_err_global > linear_vel_)
			cmd_vel.linear.x = linear_vel_;
		else
			cmd_vel.linear.x = params_.Kpx_param_*dist_err_global;
		cmd_vel.angular.z = -Kpy_ * y_err_local; 
		pub_cmd_.publish(cmd_vel);	
		// ELSE (VIDEO DRIVING)
		// PUB (self_driving/auto_mode to be True)
	}

	// 3.2.2 Arrived to the goal position
	else
	{
		const geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
		tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
		tf2::Quaternion orientation (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);
		tf2::Matrix3x3 m(orientation);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);

		if(!is_rotating_) 
		{
			cout<<"**Arrived to the goal position: "<<dist_err_global<<endl;
			goal_yaw = yaw + M_PI; // save current when start rotating
			if(goal_yaw > M_PI)
				goal_yaw -= 2*M_PI;
			else if(goal_yaw < -M_PI)
				goal_yaw += 2*M_PI;

			is_rotating_ = true;
			auto_flag.data = false;

			// To contorl with Joy
			cmd_vel.linear.x = 0.0;
			cmd_vel.linear.z = 0.0;
			pub_auto_mode_.publish(auto_flag); 
			pub_cmd_.publish(cmd_vel);
			ros::Duration(1).sleep();
		}

		// 3.2.2.1 Check whether robot should rotate
			//To rotate 180 degree from the position where the mobile robot is arrived. // goalyaw mean "arrival yaw"
		double angle_err_global = goal_yaw - yaw;  
		cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<<endl;
		if(angle_err_global > M_PI)
			angle_err_global -= 2*M_PI;
		else if(angle_err_global < -M_PI)
			angle_err_global += 2*M_PI;
		
		//cmd_vel.angular.z = -params_.Kpy_param_rot_*angle_err_global;
		double bounded_ang_err = min(abs(angle_err_global), 1.0);
		cmd_vel.angular.z = -params_.Kpy_param_rot_ * bounded_ang_err;
		cout<<"rotating ... bounded_angle_err: "<<cmd_vel.angular.z <<"angle: "<<angle_err_global<<endl;						
		
		if(abs(angle_err_global) < params_.global_angle_boundary_ || joy_driving_)
		{
			cout<<"finish rotation"<<endl;
			goal_index_++;

			//To delete adjusing_angle
			is_rotating_ = false;
			auto_flag.data = true;
			joy_driving_ = false;
		}
		//pub_cmd_.publish(cmd_vel);
		pub_auto_mode_.publish(auto_flag); 
	}
}   


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "leo_driving_node");

	LineExtractRP LineExtractRP;
	Command Command;	

	while(ros::ok()) 
	{
		ros::spinOnce();
	}

  	return 0;
}
