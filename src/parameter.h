#include <ros/ros.h>

#include <geometry_msgs/PoseStamped.h>

class VeloParameters
{
public:
    bool load(ros::NodeHandle& pnh)
    {
        ROS_INFO("parameter loading (velodyne)");
        pnh.param("obstacle_coefficient", obstacle_coefficient_ , 0.005);
	    pnh.param("front_obstacle_dist", front_obstacle_dist_, 0.1);

	    return true;
    }    
public:
    
	double obstacle_coefficient_;
	double front_obstacle_dist_;
};


class RansacParameters
{
public:
    bool load(ros::NodeHandle& pnh)
    {
        pnh.param("line_thresh", line_thresh_, 0.5);
        ROS_INFO("parameter loading (ransac)");
	    return true;
    }    
public:
    double line_thresh_;
};

class CmdParameters
{
public:
    bool load(ros::NodeHandle& pnh)
    {
        ROS_INFO("parameter loading (cmd)");
        pnh.param("adjusting_y_err_bound", adjusting_y_err_bound_, 0.05);
        pnh.param("Kpx_param", Kpx_param_, 2.0);
        pnh.param("linear_vel", linear_vel_, 0.0);
        pnh.param("Kpy_param", Kpy_param_, 1.1);
        pnh.param("Kpy_param_rot", Kpy_param_rot_, 0.01);
        pnh.param("boundary_percent", boundary_percent_, 0.02);
        pnh.param("robot_width", robot_width_, 0.45);
		
        pnh.param("global_dist_boundary", global_dist_boundary_, 0.5);
        pnh.param("global_angle_boundary", global_angle_boundary_, 0.5);
        pnh.param("mapping_mode", mapping_mode_, true);
        pnh.param("num_goals", num_goals_, 2);
        pnh.param("line_width_min", line_width_min_, 0.7);
        pnh.param("line_width_max", line_width_max_, 1.0);
	pnh.param("obs_coefficient", obs_coefficient_, 0.1);
        return true;
    }

public:

    bool mapping_mode_;
    double adjusting_y_err_bound_;

    double Kpx_param_;
    double linear_vel_;
    double Kpy_param_;
    double Kpy_param_rot_;
    double boundary_percent_; 
    double robot_width_;
	double global_dist_boundary_;
	double global_angle_boundary_;
	double line_width_min_;
	double line_width_max_;
    int num_goals_;
	double obs_coefficient_;
	
};
