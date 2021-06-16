#include "ros/ros.h"
#include "nodelet/nodelet.h"
#include <pluginlib/class_list_macros.h>

#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/Joy.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/Float32.h>
#include <std_msgs/Bool.h>
#include <geometry_msgs/Twist.h>

namespace auto_driving {

class CmdPublishNode : public nodelet::Nodelet {

    bool joy_driving_ = false; // even: auto, odd: joy control
    float obs_x_, obs_y_;
	float y_err_local_ = 0;
    float y_err_global_ = 0;

public:
	CmdPublishNode() = default;

private:
	virtual void onInit() {
		ros::NodeHandle nh = getNodeHandle();
		ros::NodeHandle nhp = getPrivateNodeHandle();

        // Configuration
        pnh.param("Kpx_param", Kpx_param_, 2.0);
        pnh.param("Kpy_param", Kpy_param_, 1.1);
        pnh.param("Kpy_param_rot", Kpy_param_rot_, 0.01);
        pnh.param("linear_vel", linear_vel_, 0.0);
        pnh.param("robot_width", robot_width_, 0.45);
        pnh.param("line_width_min", line_width_min_, 0.7);
        pnh.param("line_width_max", line_width_max_, 1.0);
	    pnh.param("obs_coefficient", obs_coefficient_, 0.5);
	    pnh.param("boundary_percent", boundary_percent_, 0.02);
        pnh.param("spare_length", spare_length_, 1.5);

        // Subscriber & Publisher
        sub_joy_ = nhp.subscribe<sensor_msgs::Joy>("/joystick", 10, &CmdPublishNode::joyCallback, this);
		sub_obs_dists = nhp.subscribe<std_msgs::Float32MultiArray> ("obstacles/obs_dists", 10, &CmdPublishNode::obsCallback, this);
        sub_aisle_ = nhp.subscribe<sensor_msgs::PointCloud2> ("aisle/points_msg", 10, &&CmdPublishNode::aisleCallback, this);
        sub_global_dist_err_ = nhp.subscribe<std_msgs::Float32>("/localization/global_dist_err", 10, &CmdPublishNode::publishCmd, this);
        //sub_global_ang_err_ = nhp.subscribe<std_msgs::Float32>("/localization/global_ang_err", 10, &CmdPublishNode::globalAngErrCallback, this);

        pub_cmd_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
	};

    void joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
    {
        //Button "B" : driving mode change -->   even: auto, odd: joy control
        if (joy_msg->buttons[1] == 1)	
        {
            cout<<"B push"<<endl;
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

    void aisleCallback(const sensor_msgs::PointCloud2& cloud_msg)
    {
        PointCloud2Ptr tmp_cloud(new PointCloud2);
        pcl_conversions::toPCL(cloud_msg, *tmp_cloud);
        PointCloudPtr data_cloud(new PointCloud); 
        pcl::fromPCLPointCloud2(*tmp_cloud, *data_cloud); 
        
        pcl::PointXYZ nearest_point = data_cloud->points[0];
        pcl::PointXYZ reference_point = data_cloud->points[1];
        pcl::PointXYZ line_start_point = data_cloud->points[2];
        pcl::PointXYZ line_end_point = data_cloud->points[3];

        y_err_local_ = reference_point.y - nearest_point.y;
        line_start_y_ = line_start_point.y;
        line_end_y_ = line_end_point.y;        
    }

    void publishCmd(const std_msgs::Float32::ConstPtr& global_err_msg)
    {}

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

	} Config;
	Config config_;
};
}
PLUGINLIB_EXPORT_CLASS(auto_driving::CmdPublishNode, nodelet::Nodelet);

