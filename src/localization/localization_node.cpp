#include <tunnel_driving/localization_node.hpp>

using namespace std;
using namespace tunnel_driving;

void LocalizationNode::onInit()
{
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &private_nh = getMTPrivateNodeHandle();
    // Configuration //
    private_nh.param("global_dist_boundary", config_.global_dist_boundary_, 0.3);
    private_nh.param("global_angle_boundary", config_.global_angle_boundary_, 0.05);

    // Subscriber & Publisher
    // sub_pose_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &LocalizationNode::poseCallback, this);
    sub_goal_ = nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 1, &LocalizationNode::setGoal, this);    
    
    pub_arrival_ = nh.advertise<std_msgs::Bool> ("/arrival", 10);
    pub_rotating_ = nh.advertise<std_msgs::Bool> ("/rotating", 10);	
}

void LocalizationNode::poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg)
{
	if (goal_count_ < 2)
	{
		ROS_INFO_ONCE("Waiting for inserting goal... ");
		return;
	}

	std_msgs::Bool arrival_flag;
	std_msgs::Bool rotating_flag;

	current_goal_ = goal_set_[goal_index_ % goal_count_];	
	cout<<"goal is set: "<<current_goal_.pose.position.x<<" "<<current_goal_.pose.position.y<<endl;	

	// 3.2 Calculate Global Error
	float x_err_global = current_goal_.pose.position.x - pose_msg->pose.pose.position.x;
	float y_err_global = current_goal_.pose.position.y - pose_msg->pose.pose.position.y;
	double dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);	

	cout << "goal (x,y): " <<"(" <<current_goal_.pose.position.x << ", " <<current_goal_.pose.position.y << ")" <<endl;		
	cout << "curr (x,y): " <<"(" <<pose_msg->pose.pose.position.x << ", " << pose_msg->pose.pose.position.y << ")" <<endl;
	cout << "distance :" << dist_err_global <<endl;
	cout <<" " <<endl;

	double goal_yaw;	
	
    // 3.2.1 Not Arrived to the goal position
	if (dist_err_global > config_.global_dist_boundary_ && !is_rotating_) 
	{
        arrival_flag.data = false;
	}

	// 3.2.2 Arrived to the goal position
	else
	{
        arrival_flag.data = true;
		std::string errMsg;
		tf::StampedTransform transform;
		tf::TransformListener tf_listener;
		if (!tf_listener.waitForTransform("/odom",  "/base_link", ros::Time(0), ros::Duration(0.5),
											ros::Duration(0.01), &errMsg)) {
			ROS_ERROR_STREAM("Pointcloud transform | Unable to get pose from TF: ");
		} else {
			try {
				tf_listener.lookupTransform("/odom",  "/base_link", ros::Time(0), transform);
			}
			catch (const tf::TransformException &e) {
				ROS_ERROR_STREAM(
						"Pointcloud transform | Error in lookupTransform of " <<  "/base_link" << " in " << "/odom");
			}
		}

		// tf::StampedTransform transform;
		// try{
		// 	listener.lookupTransform("/odom", "/base_link",  
		// 						ros::Time(0), transform);
		// }
		// catch (tf::TransformException ex){
		// 	ROS_ERROR("%s",ex.what());
		// 	ros::Duration(1.0).sleep();
		// }
		
		// const geometry_msgs::TransformStamped trans;
		// trans = tfbuf_.lookupTransform("odom", "base_link", ros::Time(0));      
		// tf2::Vector3 translation (trans.transform.translation.x, trans.transform.translation.y, trans.transform.translation.z);
		// tf2::Quaternion orientation (trans.transform.rotation.x, trans.transform.rotation.y, trans.transform.rotation.z, trans.transform.rotation.w);
		// tf2::Matrix3x3 m(orientation);
		/*
		tf2::Vector3 translation (transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
		tf2::Quaternion orientation (transform.getRotation().x(), transform.getRotation().y(), transform.getRotation().z(), transform.getRotation().w());
		tf2::Matrix3x3 m(orientation);
		double roll, pitch, yaw;
		m.getRPY(roll, pitch, yaw);
		if(!is_rotating_) 
		{
			cout<<"**Arrived to the goal position: "<<dist_err_global<<endl;
			goal_yaw = yaw + M_PI; // save current when start rotating
			if(goal_yaw > M_PI)
				goal_yaw -= 2*M_PI;
			else if(goal_yaw < -M_PI)
				goal_yaw += 2*M_PI;

			is_rotating_ = true;
			ros::Duration(1).sleep();
		}

		// 3.2.2.1 Check whether robot should rotate
			//To rotate 180 degree from the position where the mobile robot is arrived. // goalyaw mean "arrival yaw"
		double angle_err_global = goal_yaw - yaw;  
		cout<<  "start yaw: " << goal_yaw  <<", cur yaw: " << yaw<<endl;
		if(angle_err_global > M_PI)
			angle_err_global -= 2*M_PI;
		else if(angle_err_global < -M_PI)
			angle_err_global += 2*M_PI;
		
		cout<<"rotating ... bounded_angle_err: "<<angle_err_global<<endl;
		
		if(abs(angle_err_global) < config_.global_angle_boundary_)
		{
			cout<<"finish rotation"<<endl;
			goal_index_++;

			//To delete adjusing_angle
			is_rotating_ = false;
            arrival_flag.data = true;
		}
	}
    rotating_flag.data = is_rotating_;
    pub_rotating_.publish(rotating_flag);
    pub_arrival_.publish(arrival_flag);*/
	}
}

void LocalizationNode::setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg)
{
	goal_count_ ++;
	ROS_INFO("%d th goal is set", goal_count_);
	goal_set_.push_back(*click_msg);
}

