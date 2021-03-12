#include <ros/ros.h>

class Parameters
{
public:
    bool load(ros::NodeHandle& pnh)
    {
        ROS_INFO("parameter loading");
        pnh.param("linear_vel", linear_vel_, 0.0);
        pnh.param("linear_vel_rot", linear_vel_rot_, 0.01);
        pnh.param("Kpy_param", Kpy_param_, 1.1);
        pnh.param("Kpy_param_rot", Kpy_param_rot_, 0.01);
        pnh.param("boundary_percent", boundary_percent_, 0.02);
        pnh.param("robot_width", robot_width_, 0.45);
        pnh.param("side_obstacle_dist", side_obstacle_dist_, 0.2);
        pnh.param("front_obstacle_dist", front_obstacle_dist_, 0.1);
        pnh.param("obastalce_coefficient", obastalce_coefficient_, 0.1);
        pnh.param("rotation_dist_err", rotation_dist_err_, 0.1);
        pnh.param("rotation_ang_err", rotation_ang_err_, 0.15);
        pnh.param("back_rotating_ang", back_rotating_ang_, 3.0);
        pnh.param("line_thresh", line_thresh_, 0.5);

        return true;
    }

public:
    double linear_vel_;
    double Kpy_param_;
    double Kpy_param_rot_;
    double boundary_percent_; 
    double robot_width_;
    double side_obstacle_dist_;
    double front_obstacle_dist_;
    double obastalce_coefficient_;
    double rotation_dist_err_;
    double rotation_ang_err_;
    double linear_vel_rot_;
    double back_rotating_ang_;
    double line_thresh_;
};
