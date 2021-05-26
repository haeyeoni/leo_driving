// Copyright 2021, Postech, Cocel, Control Team
#ifndef SIGNLE_LOCALIZATION_H
#define SIGNLE_LOCALIZATION_H

#include <iostream>
#include <ros/ros.h>
#include <ros/publisher.h>
#include <ros/subscriber.h>

#include <std_msgs/Bool.h>
#include <sensor_msgs/Joy.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include "obs_avoid.h"

using namespace std;

class SingleLocal
{
public:
	SingleLocal():pnh_("~"), tfl_(tfbuf_) 
	{

		// Parameter check
		bool check_obstacles;
		bool mapping_mode;
		double param_global_dist_boundary, param_global_angle_boundary;
		tf2_ros::Buffer tfbuf_;
		tf2_ros::TransformListener tfl_(tfbuf_);

		pnh_.param("check_obstacles", check_obstacles, false);
		pnh_.param("mapping_mode", mapping_mode , true);
		pnh_.param("global_dist_boundary", p_global_dist_boundary_ , 0.3);
		pnh_.param("global_angle_boundary", p_global_angle_boundary_ , 0.1);
		pub_rotating_ = nh_.advertise<std_msgs::Bool> ("/is_rotating", 10);	
		pub_cmd_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
		if (check_obstacles)
		{        
		    VeloObs VeloObs;
		    if(!VeloObs.configure())
		    {  
		        ROS_ERROR("Error configuring Obs Class");
		    }
		}

		if(!mapping_mode)
		{
			sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 10, &SingleLocal::handleJoyMode, this);
			sub_amcl_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &SingleLocal::handlePose, this);
			sub_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &SingleLocal::setGoal, this);

			pub_arrival_ = nh_.advertise<std_msgs::Bool> ("/is_arrived", 10);
			pub_rotating_ = nh_.advertise<std_msgs::Bool> ("/is_rotating", 10);		
		}	
	};

	void handleJoyMode(const sensor_msgs::Joy::ConstPtr& joy_msg){
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

	void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
	{	
		goal_count_ ++;
		ROS_INFO("%d th goal is set", goal_count_);
		goal_set_.push_back(*click_msg);
	}  

	void handlePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
	{
		amcl_pose_.pose.covariance = pose_msg->pose.covariance;
		amcl_pose_.pose.pose.position = pose_msg->pose.pose.position;
		amcl_pose_.pose.pose.orientation = pose_msg->pose.pose.orientation;
		checkArrival();
	}   

	void checkArrival()
	{
        arrival_flag_.data = false;
        rotating_flag_.data = false;

        if(goal_count_ == 2) 
        {

			ROS_INFO_ONCE("Checking the Arrival state ..");
            // (1) Update Goal Position
            geometry_msgs::PoseStamped current_goal = goal_set_[goal_index_ % goal_count_];	
            tf2::Quaternion orientation (current_goal.pose.orientation.x, current_goal.pose.orientation.y, current_goal.pose.orientation.z, current_goal.pose.orientation.w);
            tf2::Matrix3x3 m(orientation);
            double goal_roll, goal_pitch, goal_yaw;
            m.getRPY(goal_roll, goal_pitch, goal_yaw);
            
            // (2) Calculate Global Error
            float x_err_global = current_goal.pose.position.x - amcl_pose_.pose.pose.position.x;
            float y_err_global = current_goal.pose.position.y - amcl_pose_.pose.pose.position.y;
            double dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);		
            std::cout << "goal (x,y): " <<"(" <<current_goal.pose.position.x << ", " <<current_goal.pose.position.y << ")" <<std::endl;		
            std::cout << "curr (x,y): " <<"(" <<amcl_pose_.pose.pose.position.x << ", " << amcl_pose_.pose.pose.position.y << ")" <<std::endl;
            std::cout << "distance :" << dist_err_global <<std::endl;
            std::cout <<" " <<std::endl;

            if (dist_err_global < p_global_dist_boundary_) 
            // (3) If arrived to the goal position
            {
                std::cout<<"**Arrived to the goal position: "<<dist_err_global<<std::endl;	                
                arrival_flag_.data = true;

                // (4) Check whether robot should rotate
                const geometry_msgs::TransformStamped trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
                tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
                tf2::Quaternion orientation (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);
                tf2::Matrix3x3 m(orientation);
                double roll, pitch, yaw;
                m.getRPY(roll, pitch, yaw);
                double angle_err_global = goal_yaw - yaw;
                if(angle_err_global > M_PI)
                    angle_err_global -= 2*M_PI;
                else if(angle_err_global < -M_PI)
                    angle_err_global += 2*M_PI;

                if(abs(angle_err_global) < p_global_angle_boundary_)
                {
                    std::cout<<"finish rotation"<<std::endl;
                    goal_index_++;
                    rotating_flag_.data = false; // finished rotation
					arrival_flag_.data = false;
                }
                else
                {
                    rotating_flag_.data = true;
                }
            }    

            pub_arrival_.publish(arrival_flag_);
            pub_rotating_.publish(rotating_flag_); 
        }
	}
	
	// class variables
    ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
        
    ros::Subscriber sub_joy_, sub_amcl_, sub_goal_;
    ros::Publisher pub_arrival_, pub_rotating_, pub_cmd_;

    // TF 
    tf2_ros::Buffer tfbuf_;
    tf2_ros::TransformListener tfl_;
    tf2_ros::TransformBroadcaster tfb_;

	int goal_count_;
	int goal_index_ = 0;

	std_msgs::Bool arrival_flag_, rotating_flag_;
	std::vector<geometry_msgs::PoseStamped> goal_set_;
	geometry_msgs::PoseWithCovarianceStamped amcl_pose_;
	double p_global_dist_boundary_, p_global_angle_boundary_;
 	bool joy_driving_ = false;
};

#endif

