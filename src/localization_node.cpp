#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Float32MultiArray.h>

namespace auto_driving {

class LocalizationNode : public nodelet::Nodelet {

public:
	LocalizationNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();
		
		// Configuration //
		nhp.param("global_dist_boundary", config_.global_dist_boundary_, 0.3);
		nhp.param("global_angle_boundary", config_.global_angle_boundary_, 0.05);

		sub_goal_ = nhp.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &LocalizationNode::setGoal, this);    
		sub_pose_ = nhp.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &LocalizationNode::poseCallback, this);
		
		pub_localization_ = nhp.advertise<std_msgs::Float32MultiArray>("/localization_data", 10); 
		// [0]: global dist error, [1]: global angle error, [2]: arrival flag, [3]: rotating flag
	};
	
	void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
	{
		goal_count_ ++;
		ROS_INFO("%d th goal is set", goal_count_);
		goal_set_.push_back(*click_msg);
	}

	void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
	{
		std_msgs::Float32MultiArray localization_msgs;
		localization_msgs.data.clear();
		if (goal_count_ < 2)
		{
			ROS_INFO_ONCE("Waiting for inserting goal... ");
			localization_msgs.data.push_back(10000); // global_dist_err
			localization_msgs.data.push_back(10000); // global_ang_err
			localization_msgs.data.push_back(true); // is_arrived -> Not moving
			localization_msgs.data.push_back(false); // is_rotating
			pub_localization_.publish(localization_msgs);
			return;
		}
		bool is_arrived = false;
		current_goal_ = goal_set_[goal_index_ % goal_count_];	
		ROS_INFO_ONCE("Goal is set: %d, %d", current_goal_.pose.position.x, current_goal_.pose.position.y);

		// 1. Calculate Global Error
		float global_x_err = current_goal_.pose.position.x - pose_msg->pose.pose.position.x;
		float global_y_err = current_goal_.pose.position.y - pose_msg->pose.pose.position.y;
		double global_dist_err = sqrt(global_x_err*global_x_err + global_y_err*global_y_err);	
		double global_ang_err;
		std::cout << "goal (x,y): " <<"(" <<current_goal_.pose.position.x << ", " <<current_goal_.pose.position.y << ")" <<std::endl;		
		std::cout << "curr (x,y): " <<"(" <<pose_msg->pose.pose.position.x << ", " << pose_msg->pose.pose.position.y << ")" <<std::endl;
		std::cout << "distance :" << global_dist_err <<std::endl;
		std::cout <<" " <<std::endl;
		
		// 2.1 Not Arrived to the goal position
		if (global_dist_err > config_.global_dist_boundary_ && !is_rotating_) 
		{
			is_arrived = false;
			global_ang_err = M_PI;
		}
	
		// 2.2 Arrived to the goal position
		else
		{
			is_arrived = true;
			tf::StampedTransform transform;
			tf::TransformListener tf_listener;
			if (!tf_listener.waitForTransform("/odom", "/base_link", ros::Time(0), ros::Duration(0.5), ros::Duration(0.01))) 
			{
				ROS_ERROR("Unable to get pose from TF");
				return;
			}
			try 
			{
				tf_listener.lookupTransform("/odom", "/base_link", ros::Time(0), transform);
			}
			catch (const tf::TransformException &e) {
				ROS_ERROR("%s",e.what());
			} 				
			// angle
			tf2::Quaternion orientation (transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
			tf2::Matrix3x3 m(orientation);
			double roll, pitch, yaw;
			m.getRPY(roll, pitch, yaw);
		
			if(!is_rotating_) 
			{
				std::cout<<"**Arrived to the goal position: "<<global_dist_err<<std::endl;
				goal_yaw = yaw + M_PI; // save current when start rotating
				if(goal_yaw > M_PI)
					goal_yaw -= 2*M_PI;
				else if(goal_yaw < -M_PI)
					goal_yaw += 2*M_PI;
				is_rotating_ = true;
				ros::Duration(1).sleep();
			}

			// 2.2.1 Check whether robot should rotate
				//To rotate 180 degree from the position where the mobile robot is arrived. // goalyaw mean "arrival yaw"
			global_ang_err = goal_yaw - yaw;  
			std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<<std::endl;
			if(global_ang_err > M_PI)
				global_ang_err -= 2*M_PI;
			else if(global_ang_err < -M_PI)
				global_ang_err += 2*M_PI;
			
			std::cout<<"rotating ... bounded_angle_err: "<<global_ang_err<<std::endl;
			if(abs(global_ang_err) < config_.global_angle_boundary_)
			{
				std::cout<<"finish rotation"<<std::endl;
				goal_index_++;
				is_rotating_ = false;
				is_arrived = false;
			}
		}
		localization_msgs.data.push_back(global_dist_err);
		localization_msgs.data.push_back(global_ang_err);
		localization_msgs.data.push_back(is_arrived);
		localization_msgs.data.push_back(is_rotating_);
		pub_localization_.publish(localization_msgs);
	}

private:
	ros::Subscriber sub_pose_;
	ros::Subscriber sub_goal_;
	ros::Publisher pub_localization_;
	
	bool is_rotating_ = false;

	// GOAL
	int goal_index_ = 0;
	int goal_count_ = 0;    
	double goal_yaw = M_PI;	
		
	std::vector<geometry_msgs::PoseStamped> goal_set_;
	geometry_msgs::PoseStamped current_goal_;

	/** configuration parameters */
	typedef struct
	{
		double global_dist_boundary_;
		double global_angle_boundary_;
	} Config;
	Config config_;

};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::LocalizationNode, nodelet::Nodelet);

