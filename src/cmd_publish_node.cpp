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
#include <algorithm>

namespace auto_driving {

class CmdPublishNode : public nodelet::Nodelet {
    bool joy_driving_ = false; // even: auto, odd: joy control
    // Obs
    float obs_x_ = 10000;
    float obs_y_ = 10000;	
    bool temp_is_obs_in_aisle = false;
    double spare_length = 0;
    // Amcl
    float global_dist_err_ = 0;
    float global_ang_err_ = 0;    
    
    // Aisle
    float line_start_y_ = -30; 
    float line_end_y_ = 30;
    float ref_y_ = 0;
    float near_y_ = 0;
    
    // 
    bool is_rotating_ = false;
    bool is_arrived_ = true;
    

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
	    nhp.param("front_obs", config_.front_obs_, 0.6);
	    nhp.param("boundary_percent", config_.boundary_percent_, 0.02);
        nhp.param("spare_length", config_.spare_length_, 1.5);
        nhp.param("amcl_driving", config_.amcl_driving_, false);
        nhp.param("check_obstacles", config_.check_obstacles_, false);

        // // Subscriber & Publisher
        sub_joy_ = nhp.subscribe<sensor_msgs::Joy>("/joystick", 1, &CmdPublishNode::joyCallback, this);
		sub_obs_dists_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/obs_dists", 10, &CmdPublishNode::obsCallback, this);
        sub_aisle_ = nhp.subscribe<sensor_msgs::PointCloud2> ("/aisle_points", 10, &CmdPublishNode::aisleCallback, this);    
        sub_localization_ = nhp.subscribe<std_msgs::Float32MultiArray> ("/localization_data", 10, &CmdPublishNode::localDataCallback, this);
    
        sub_driving_ = nhp.subscribe<std_msgs::Bool> ("/lidar_driving", 10, &CmdPublishNode::publishCmd, this);
        
        pub_cmd_ = nhp.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
	};

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        //Button "B" : driving mode change -->   even: auto, odd: joy control
        if (joy_msg->buttons[1] == 1)	
        {
            std::cout<<"B push"<<std::endl;
            joy_driving_ = !joy_driving_;
            ros::Duration(0.5).sleep();
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

        near_y_ = data_cloud->points[0].y;
        ref_y_ = data_cloud->points[1].y;
        line_start_y_ = data_cloud->points[2].y;
        line_end_y_ = data_cloud->points[3].y;        
    }

    void localDataCallback(const std_msgs::Float32MultiArray::ConstPtr& local_msgs)
    {
        global_dist_err_ = local_msgs->data[0]; 
        global_ang_err_ = local_msgs->data[1];
        is_arrived_ = local_msgs->data[2];
        is_rotating_ = local_msgs->data[3];
    }

    void publishCmd(const std_msgs::Bool::ConstPtr &driving_start)
    {
        //// 1. Joystick Driving
        if(joy_driving_) 
		    return;
        
        //// 2. Autonomous Driving
        geometry_msgs::Twist cmd_vel;
        double y_err_local = ref_y_ - near_y_;
        // 2.1 Check Obstacles
        if (config_.check_obstacles_)
        {
            temp_is_obs_in_aisle = true;
            float line_length = line_end_y_ - line_start_y_;
            float left_boundary = line_start_y_ - (line_length * config_.boundary_percent_ + 0.5 * config_.robot_width_);
        	float right_boundary = line_end_y_ + (line_length * config_.boundary_percent_ + 0.5 * config_.robot_width_);
            bool is_obs_in_aisle = obs_y_ > line_end_y_ && obs_y_ < line_start_y_;
            spare_length = 0;
            // (0) Front Obstacle Update
		
            if (obs_x_ < config_.front_obs_ && abs(obs_y_) < config_.robot_width_/4 && !is_rotating_)
            {
		cmd_vel.linear.x = 0.0;
                cmd_vel.linear.z = 0.0;
                pub_cmd_.publish(cmd_vel);
                return;
            }
            /*
		// (1) Right Obstacle Update	
            else if(obs_y_ < 0 && obs_y_ > -1 && obs_x_ < 0.6)
            {	
                std::cout << "Right obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<std::endl;
                float shift = config_.obs_coefficient_*(line_end_y_ - obs_y_);
                y_err_local = (near_y_ + shift > left_boundary) ? left_boundary - near_y_ : y_err_local + shift;
            }
            // (2) Left Obstacle Update 
            else if(obs_y_ > 0 && obs_y_ < 1 && obs_x_ < 0.6)
            {
                std::cout << "Left obstacle is detected, distance = " << obs_y_ << ", x = " <<  obs_x_<<std::endl;
                float shift = config_.obs_coefficient_*(line_start_y_ - obs_y_);
                y_err_local = (near_y_ + shift < right_boundary) ? right_boundary - near_y_ : y_err_local + shift;
            }
            // (3) After obs disappear, go further 'spare_length'
            if(is_obs_in_aisle != temp_is_obs_in_aisle)
            { 
                spare_length += config_.linear_vel_ * 0.1;
                y_err_local = 0;  
                std::cout<< "straight foward of spare distance" <<std::endl;
                if(spare_length > config_.spare_length_)
                {
                    spare_length = 0;
                    temp_is_obs_in_aisle = false;
                    std::cout<<"spare finish"<<std::endl;
                }
            }
	*/			
	    }

        // 2.2 Check Global Pose 
        if (!config_.amcl_driving_) // No amcl (Mapping Mode)
        {
            cmd_vel.linear.x = config_.linear_vel_;
	    cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local; 
        }
        else // AMCL Mode
        {
            // 2.2.1 Not Arrived to the goal position
            if (is_rotating_)
            {
                double bounded_ang_err = std::min(abs(double(global_ang_err_)), 1.0);
                cmd_vel.angular.z = -config_.Kpy_param_rot_ * bounded_ang_err;
                cmd_vel.linear.x = 0.0;
            }
            else if(is_arrived_)
            {
                cmd_vel.linear.x = 0.0;
                cmd_vel.linear.z = 0.0;
                ros::Duration(1).sleep();
            }
            else 
            {
                cmd_vel.linear.x = config_.linear_vel_;
                cmd_vel.angular.z = -config_.Kpy_param_ * y_err_local; 
            }
        }
        pub_cmd_.publish(cmd_vel);
    }

private:
	// Publisher & Subscriber
	ros::Subscriber sub_joy_;
    ros::Subscriber sub_obs_dists_;
	ros::Subscriber sub_aisle_;
	ros::Subscriber sub_localization_;
	ros::Subscriber sub_driving_;
    
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
        double front_obs_;
        double spare_length_;
        double boundary_percent_;;
        double line_width_min_;
        double line_width_max_;
        bool amcl_driving_;
        bool check_obstacles_;
	} Config;
	Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::CmdPublishNode, nodelet::Nodelet);

