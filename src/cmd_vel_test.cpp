// Copyright 2020, Postech, Cocel, Control Team

#include "cmd_vel_test.h"
#include "serial.hpp"

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

// 2. CALCULATE AISLE LINE LENGTH AND CENTER
	pcl::PointXYZ nearest_point = data_cloud->points[0];
	pcl::PointXYZ reference_point = data_cloud->points[1];

	float x_err_local = reference_point.x - nearest_point.x; 
	float y_err_local = reference_point.y - nearest_point.y;
	
}

void Command::handlePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
	if (goal_count_ < 2)
	{
		ROS_INFO_ONCE("Waiting for inserting goal... ");
		return;
	}
	current_goal_ = goal_set_[goal_index_ % goal_count_];	
	cout<<"goal is set: "<<current_goal_.pose.position.x<<" "<<current_goal_.pose.position.y<<endl;	
	

	// 3.2 Calculate Global Error
	x_err_global = current_goal_.pose.position.x - pose_msg->pose.pose.position.x;
	y_err_global = current_goal_.pose.position.y - pose_msg->pose.pose.position.y;
	double dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);	

	cout << "goal (x,y): " <<"(" <<current_goal_.pose.position.x << ", " <<current_goal_.pose.position.y << ")" <<endl;		
	cout << "curr (x,y): " <<"(" <<apose_msg->pose.pose.position.x << ", " << pose_msg->pose.pose.position.y << ")" <<endl;
	cout << "distance :" << dist_err_global <<endl;
	cout <<" " <<endl;
	

	double goal_roll, goal_pitch;	
	// 3.2.1 Not Arrived to the goal position
	if (dist_err_global > params_.global_dist_boundary_ || !is_rotating_) 
	{
		if(params_.Kpx_param_*dist_err_global > linear_vel)
			cmd_vel.linear.x = linear_vel;
		else
			cmd_vel.linear.x = params_.Kpx_param_*dist_err_global;
		cmd_vel.angular.z = -Kpy * y_err_local_; 

		pub_cmd_.publish(cmd_vel);	
	}

	// 3.2.2 Arrived to the goal position
	else
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

		// 3.2.2.1 Check whether robot should rotate
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
		
		// 3.2.2.2 If finish rotation,	 
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
