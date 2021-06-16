#ifndef LOCALIZATION_NODE_H
#define LOCALIZATION_NODE_H

#include <ros/ros.h>
 
#include <sensor_msgs/Joy.h>
#include <std_msgs/Bool.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_sensor_msgs/tf2_sensor_msgs.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <nodelet/nodelet.h>
namespace tunnel_driving{
	
class LocalizationNode : public nodelet::Nodelet
{
public:
	LocalizationNode():tfl_(tfbuf_) {};

	virtual void onInit();
	void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg);	
    void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg);
	void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg);
	
private:
	// Publisher & Subscriber
    ros::Subscriber sub_pose_;
    ros::Subscriber sub_goal_;
	ros::Publisher pub_arrival_;
	ros::Publisher pub_rotating_;

    // TF 
	tf2_ros::Buffer tfbuf_;
    tf2_ros::TransformListener tfl_;

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
#endif