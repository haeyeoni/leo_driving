#include <ros/ros.h>

class Parameters
{
public:
    bool load(ros::NodeHandle& pnh)
    {
        ROS_INFO("parameter loading");
        pnh.param("linear_vel", linear_vel_, 0.0);
        pnh.param("Kpy_param", Kpy_param_, 1.1);
        pnh.param("boundary_percent", boundary_percent_, 0.02);
        pnh.param("robot_width", robot_width_, 0.45);
        pnh.param("side_obstacle_dist", side_obstacle_dist_, 0.2);
        pnh.param("front_obstacle_dist", front_obstacle_dist_, 0.1);
        pnh.param("obastalce_coefficient", obastalce_coefficient_, 0.1);

        return true;
    }

public:
    double linear_vel_;
    double Kpy_param_;
    double boundary_percent_; 
    double robot_width_;
    double side_obstacle_dist_;
    double front_obstacle_dist_;
    double obastalce_coefficient_;
};