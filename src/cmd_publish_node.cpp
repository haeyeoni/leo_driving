#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>

namespace auto_driving {

class CmdPublishNode : public nodelet::Nodelet {

    bool joy_driving_ = false; // even: auto, odd: joy control
    float obs_x_, obs_y_;
	float y_err_local_ = 0;
    float y_err_global_ = 0;
    float line_start_y_, line_end_y_;
    bool temp_is_obs_in_aisle;
public:
	CmdPublishNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();

        // Configuration
        nhp.param("Kpx_param", config_.Kpx_param_, 2.0);
        nhp.param("Kpy_param", config_.Kpy_param_, 1.1);
        nhp.param("Kpy_param_rot", config_.Kpy_param_rot_, 0.01);
        nhp.param("linear_vel", config_.linear_vel_, 0.0);
        nhp.param("robot_width", config_.robot_width_, 0.45);
        nhp.param("line_width_min", config_.line_width_min_, 0.7);
        nhp.param("line_width_max", config_.line_width_max_, 1.0);
	    nhp.param("obs_coefficient", config_.obs_coefficient_, 0.5);
	    nhp.param("boundary_percent", config_.boundary_percent_, 0.02);
        nhp.param("spare_length", config_.spare_length_, 1.5);
        nhp.param("local_driving", config_.local_driving_, false);
        nhp.param("check_obstacles", config_.check_obstacles_, false);

        // // Subscriber & Publisher
        sub_joy_ = nhp.subscribe<sensor_msgs::Joy>("/joystick", 10, &CmdPublishNode::joyCallback, this);
		sub_obs_dists_ = nhp.subscribe<std_msgs::Float32MultiArray> ("obstacles/obs_dists", 10, &CmdPublishNode::obsCallback, this);
        sub_aisle_ = nhp.subscribe<sensor_msgs::PointCloud2> ("aisle/points_msg", 10, &CmdPublishNode::aisleCallback, this);
        sub_global_dist_err_ = nhp.subscribe<std_msgs::Float32>("/localization/global_dist_err", 10, &CmdPublishNode::publishCmd, this);
        //sub_global_ang_err_ = nhp.subscribe<std_msgs::Float32>("/localization/global_ang_err", 10, &CmdPublishNode::globalAngErrCallback, this);

        pub_cmd_ = nhp.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
	};

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        //Button "B" : driving mode change -->   even: auto, odd: joy control
        if (joy_msg->buttons[1] == 1)	
        {
            std::cout<<"B push"<<std::endl;
            joy_driving_ = !joy_driving_;
            ros::Duration(1).sleep();
        }
    }

	void obsCallback(const std_msgs::Float32MultiArray::ConstPtr& dists_msg)
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

    void aisleCallback(const sensor_msgs::PointCloud2::ConstPtr& cloud_msg)
    {
        pcl::PCLPointCloud2* cloud = new pcl::PCLPointCloud2; 
        pcl::PCLPointCloud2ConstPtr cloudPtr(cloud);

        // Convert to PCL data type
        pcl_conversions::toPCL(*cloud_msg, *cloud);
        pcl::PointCloud<pcl::PointXYZ>::Ptr data_cloud(new pcl::PointCloud<pcl::PointXYZ>); 
        pcl::fromPCLPointCloud2(*cloud, *data_cloud); 
        
        pcl::PointXYZ nearest_point = data_cloud->points[0];
        pcl::PointXYZ reference_point = data_cloud->points[1];
        pcl::PointXYZ line_start_point = data_cloud->points[2];
        pcl::PointXYZ line_end_point = data_cloud->points[3];

        y_err_local_ = reference_point.y - nearest_point.y;
        line_start_y_ = line_start_point.y;
        line_end_y_ = line_end_point.y;        
    }

    void publishCmd(const std_msgs::Float32::ConstPtr& global_err_msg)
    {
        if(joy_driving_) // joystick mode
		    return;
        
        geometry_msgs::Twist cmd_vel;
        
        // // Check Obstacles
        // if (config_.check_obstacles_)
        // {
        //     temp_is_obs_in_aisle = true;
        //     float line_length = line_end_y_ - line_start_y_;
        //     float left_boundary = line_start_y_ - (line_length * config_.boundary_percent_ + 0.5 * config_.robot_width_);
        // 	float right_boundary = line_end_y_ + (line_length * config_.boundary_percent_ + 0.5 * config_.robot_width_);
        //     bool is_obs_in_aisle = obs_y_ > line_end_y && obs_y_ < line_start_y;
        //     spare_length = 0;
        //     // 1. Right Obstacle Update	
        //     if(obs_y_ < 0 && obs_y_ > -1 && obs_x_ < 0.6)
        //     {	
        //         std::cout << "Right obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<std::endl;
        //         float shift = params_.obs_coefficient_*(line_end-obs_y_);
        //         y_err_local = (nearest_point.y + shift > left_boundary) ? left_boundary - nearest_point.y : y_err_local + shift;
        //     }
        //     // 2. Left Obstacle Update 
        //     else if(obs_y_ > 0 && obs_y_ < 1 && obs_x_ < 0.6)
        //     {
        //         std::cout << "Left obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<std::endl;
        //         float shift = params_.obs_coefficient_*(line_start-obs_y_);
        //         y_err_local = (nearest_point.y + shift < right_boundary) ? right_boundary - nearest_point.y : y_err_local + shift;
        //     }
        //     // After obs disappear, go further 'spare_length'
        //     if(is_obs_in_aisle != temp_is_obs_in_aisle)
        //     { 
        //         spare_length += linear_vel * 0.1;
        //         y_err_local = 0;  
        //         std::cout<< "straight foward of spare distance" <<std::endl;
        //         if(spare_length > params_.spare_length_)
        //         {
        //             spare_length = 0;
        //             temp_is_obs_in_aisle = false;
        //             std::cout<<"spare finish"<<std::endl;
        //         }
        //     }
			
	    // }
            
        // if (config_.local_driving_) // No amcl
        // {
        //     cmd_vel.linear.x = config_.linear_vel_;
        //     cmd_vel.angular.z = -config_.Kpy_ * y_err_local_; 
        //     pub_cmd_.publish(cmd_vel);
        // }

        // if (dist_err_global > params_.global_dist_boundary_ && !is_rotating_) 
        // {
        //     // TODO : ADD CONDITION
        //     /// IF (RP DRIVING)
        //     if(params_.Kpx_param_*dist_err_global > linear_vel_)
        //         cmd_vel.linear.x = linear_vel_;
        //     else
        //         cmd_vel.linear.x = params_.Kpx_param_*dist_err_global;
        //     cmd_vel.angular.z = -Kpy_ * y_err_local_; 
        //     pub_cmd_.publish(cmd_vel);	
        //     // ELSE (VIDEO DRIVING)
        //     // PUB (self_driving/auto_mode to be True)
        // }
        
    }

private:
	// Publisher & Subscriber
	ros::Subscriber sub_joy_;
    ros::Subscriber sub_obs_dists_;
	ros::Subscriber sub_aisle_;
    ros::Subscriber sub_global_dist_err_;
    ros::Subscriber sub_global_ang_err_;
    
	ros::Publisher pub_cmd_;

	/** configuration parameters */
	typedef struct
	{
		double Kpx_param_;
		double Kpy_param_;
		double Kpy_param_rot_;
        double linear_vel_;
        double robot_width_;
        double obs_coefficient_;
        double boundary_percent_;
        double spare_length_;
        double line_width_min_;
        double line_width_max_;
        bool local_driving_;
        bool check_obstacles_;
	} Config;
	Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::CmdPublishNode, nodelet::Nodelet);

