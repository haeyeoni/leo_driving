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
		
		pub_arrival_ = nhp.advertise<std_msgs::Bool> ("localization/arrival", 10);
		pub_rotating_ = nhp.advertise<std_msgs::Bool> ("localization/rotating", 10);	
		pub_global_dist_err_ = nhp.advertise<std_msgs::Float32> ("localization/global_dist_err", 10);	
		pub_global_angle_err_ = nhp.advertise<std_msgs::Float32> ("localization/global_ang_err", 10);	
	};
	
	void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
	{
		goal_count_ ++;
		ROS_INFO("%d th goal is set", goal_count_);
		goal_set_.push_back(*click_msg);
	}

	void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
	{
		if (goal_count_ < 2)
		{
			ROS_INFO_ONCE("Waiting for inserting goal... ");
			return;
		}
		
		std_msgs::Bool arrival_flag;
		std_msgs::Bool rotating_flag;
		std_msgs::Float32 global_dist_err_msg;
		std_msgs::Float32 global_ang_err_msg;

		current_goal_ = goal_set_[goal_index_ % goal_count_];	
		std::cout<<"goal is set: "<<current_goal_.pose.position.x<<" "<<current_goal_.pose.position.y<<std::endl;	

		// 1. Calculate Global Error
		float x_err_global = current_goal_.pose.position.x - pose_msg->pose.pose.position.x;
		float y_err_global = current_goal_.pose.position.y - pose_msg->pose.pose.position.y;
		double dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);	

		global_dist_err_msg.data = dist_err_global;
		pub_global_dist_err_.publish(global_dist_err_msg);

		std::cout << "goal (x,y): " <<"(" <<current_goal_.pose.position.x << ", " <<current_goal_.pose.position.y << ")" <<std::endl;		
		std::cout << "curr (x,y): " <<"(" <<pose_msg->pose.pose.position.x << ", " << pose_msg->pose.pose.position.y << ")" <<std::endl;
		std::cout << "distance :" << dist_err_global <<std::endl;
		std::cout <<" " <<std::endl;

		double goal_yaw;	
		
		// 2.1 Not Arrived to the goal position
		if (dist_err_global > config_.global_dist_boundary_ && !is_rotating_) 
		{
			arrival_flag.data = false;
		}
	
		// 2.2 Arrived to the goal position
		else
		{
			arrival_flag.data = true;
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
				std::cout<<"**Arrived to the goal position: "<<dist_err_global<<std::endl;
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
			double angle_err_global = goal_yaw - yaw;  
			std::cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<<std::endl;
			if(angle_err_global > M_PI)
				angle_err_global -= 2*M_PI;
			else if(angle_err_global < -M_PI)
				angle_err_global += 2*M_PI;
			
			std::cout<<"rotating ... bounded_angle_err: "<<angle_err_global<<std::endl;
			global_ang_err_msg.data = angle_err_global;
			pub_global_angle_err_.publish(global_ang_err_msg);

			if(abs(angle_err_global) < config_.global_angle_boundary_)
			{
				std::cout<<"finish rotation"<<std::endl;
				goal_index_++;
				is_rotating_ = false;
			}
		}
		rotating_flag.data = is_rotating_;
		pub_rotating_.publish(rotating_flag);
		pub_arrival_.publish(arrival_flag);
	}

private:
	ros::Subscriber sub_pose_;
	ros::Subscriber sub_goal_;
	ros::Publisher pub_arrival_;
	ros::Publisher pub_rotating_;
	ros::Publisher pub_global_dist_err_;
	ros::Publisher pub_global_angle_err_;
	
	bool joy_driving_ = false; // even: auto, odd: joy control
	bool is_rotating_ = false;

	// GOAL
	int goal_index_ = 0;
	int goal_count_ = 0;    
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

