#include <tunnel_driving/aisle_detect_node.hpp>
#include <pluginlib/class_list_macros.hpp>


PLUGINLIB_EXPORT_CLASS(LocalizationNode, nodelet::Nodelet)

void LocalizationNode::onInit()
{
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &private_nh = getMTPrivateNodeHandle();
    
    // Configuration //
    private_nh.param("line_thresh", config_.line_thresh_, 0.5);
    private_nh.param("line_thresh", config_.aisle_width_, 0.6);

    // Subscriber & Publisher
    sub_amcl_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &LocalizationNode::poseCallback, this);
    sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("/joystick", 10, &LocalizationNode::joyCallback, this);
    sub_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &Command::setGoal, this);    
    pub_arrival_ = nh_.advertise<std_msgs::Bool> ("navigation/arrival", 10);
    pub_auto_mode_ = nh_.advertise<std_msgs::Bool> ("self_driving/auto_mode", 10);	
}

void LocalizationNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{

}
void LocalizationNode::setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
{

}
void LocalizationNode::joyCallback(const sensor_msgs::Joy::ConstPtr& joy_msg)
{

}
