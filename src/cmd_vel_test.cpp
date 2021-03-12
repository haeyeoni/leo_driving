// Copyright 2020, Postech, Cocel, Control Team

#include "cmd_vel_test.h"
#include <cmath>
#include "parameter.h"
using namespace std;

float LINE_START, LINE_END;
bool CHECK_LINE = false;

class LineExtractRP
{
public:
    LineExtractRP() {
		this->sub_scan_ = this->nh_.subscribe("/up_scan", 10, &LineExtractRP::lineExtract, this);
		this->pub_line_ = this->nh_.advertise<sensor_msgs::PointCloud2>("cluster_line", 10);
		this->pub_nearest_ = this->nh_.advertise<sensor_msgs::PointCloud2> ("nearest_point", 10);
	 	this->pub_ref_ = this->nh_.advertise<sensor_msgs::PointCloud2> ("reference_point", 10);
		this->pub_points_ = this->nh_.advertise<sensor_msgs::PointCloud2> ("points_msg", 10);
    };

    void lineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in) 
	{
		boost::recursive_mutex::scoped_lock ransac_lock(scope_mutex_);
			
		float Width = 0.6; //<- Data to be cropped (aisle width)
		// Messages to be published
		sensor_msgs::PointCloud2 nearest_point;
		sensor_msgs::PointCloud2 reference_point;      
		sensor_msgs::PointCloud2 points_msg;
		sensor_msgs::PointCloud2 points_line;

		// Message data before converted to the ROS messages
		PointCloud closest;
		PointCloud filtered_cloud;
		PointCloud point_set;
		PointCloud cluseter_line;

		sensor_msgs::PointCloud2 cloud_temp; // <- temporary point cloud to temporaly save the input point cloud     
		PointCloudPtr cloud(new PointCloud); // <- data cloud
		PointCloud2Ptr cloud2(new PointCloud2); // <- data cloud2 (converted from the cloud)
		PointCloudPtr cloud_inrange(new PointCloud); // <- cropped cloud
		
		// DATA TYPE CONVERSIONS: LaserScan (scan_in) -> PointCloud2 (cloud2)
		projector_.projectLaser(*scan_in, cloud_temp);
		pcl_conversions::toPCL(cloud_temp, *cloud2); 
		pcl::fromPCLPointCloud2(*cloud2, *cloud); 

		// CROP POINTCLOUD (cloud -> cloud_inrange) 
		pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ> ());
			// set condition
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -Width)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, Width)));
		range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.0)));
			// conditional removal
		pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
		condrem.setInputCloud(cloud);
		condrem.setCondition(range_condition);
		condrem.setKeepOrganized(true);
		condrem.filter(*cloud_inrange);

		if (!cloud_inrange->size())
		{
			ROS_WARN("all points are cropped");
			return;
		}
		// EXTRACT LINE (RANSAC ALGORITHM): cloud_inragne changed 
				pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
		pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		pcl::ExtractIndices<pcl::PointXYZ> extract;
		seg.setOptimizeCoefficients(true);
		seg.setModelType(pcl::SACMODEL_LINE); // <- extract model setting
		seg.setMethodType(pcl::SAC_RANSAC);
		seg.setDistanceThreshold(0.5); // <- threshold (line width)
		seg.setInputCloud(cloud_inrange); 
		seg.segment(*inliers, *coefficients);
		extract.setInputCloud(cloud_inrange);
		extract.setIndices(inliers);
		extract.setNegative(false); //<- if true, it returns point cloud except the line.
		extract.filter(*cloud_inrange);
		
		// CENTER LINE CLUSTER IS EXTRACTED AMONG MULTIPLE LINE CLUSTERS 
			// clustering... 
		pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cluster(new pcl::search::KdTree<pcl::PointXYZ>);
		tree_cluster->setInputCloud(cloud_inrange);
		std::vector<pcl::PointIndices> cluster_indices;
		pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
		ec.setInputCloud(cloud_inrange);
		ec.setClusterTolerance(0.05); // <- If the two points have distance bigger than this tolerance, then points go to different clusters. 
		ec.setMinClusterSize(30); 
		ec.setMaxClusterSize(800);;
		ec.setSearchMethod(tree_cluster);
		ec.extract(cluster_indices);		

		// extract first clustering (center cluster)
		int j = 0;
		std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
		pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
		for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
		{
			if (j == 0) {
				for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
				{		
					cloud_cluster->points.push_back (cloud_inrange->points[*pit]);
				}
				cloud_cluster->width = cloud_cluster->points.size ();
				cloud_cluster->height = 1;
				cloud_cluster->is_dense = true;
			}
			j++;
		}
		
	// FIND NEAREST POINT FROM THE ORIGIN ( => /nearest_point)
		if((*cloud_cluster).size()>0)
		{
			pcl::PointXYZ origin(0, 0, 0);	
			pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
			tree_->setInputCloud(cloud_cluster);
			std::vector<int> nn_indices(1); 
			std::vector<float> nn_dists(1);
			tree_->nearestKSearch(origin, 1, nn_indices, nn_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
			
			closest.push_back(cloud_cluster->points[nn_indices[0]]); 
			point_set.push_back(closest[0]);
		// CALCULATE THE AVERAGE COORDINATES FROM THE CENTER LINE( => /reference_point)
			float current_x = cloud_cluster->points[nn_indices[0]].x;
			float current_y = cloud_cluster->points[nn_indices[0]].y;
			float threshold = 0.5;
			float threshold_y = 0.5;
			float sum_x = 0;
			float sum_y = 0;
			int num_points = 0;

			// Update Line min & Line max
			LINE_START = 1000;
			LINE_END = 0;

			for (int i = 0; i < (*cloud_cluster).size(); i++) 
			{
				filtered_cloud.push_back(cloud_cluster->points[i]);
				sum_x += cloud_cluster->points[i].x;
				sum_y += cloud_cluster->points[i].y;
				num_points ++;
				if (LINE_START > cloud_cluster->points[i].y) 
					LINE_START = cloud_cluster->points[i].y;
				if (LINE_END < cloud_cluster->points[i].y)
					LINE_END = cloud_cluster->points[i].y;
			}
			CHECK_LINE = true;

			PointCloud reference_cloud;
			pcl::PointXYZ reference (sum_x / (float)num_points, sum_y / (float)num_points, 0);	
					
			reference_cloud.push_back(reference);
			point_set.push_back(reference);

		// FIND LINE END AND LINE START
			pcl::PointXYZ left_infinite(0, -10000, 0);	
			pcl::KdTree<pcl::PointXYZ>::Ptr tree_2(new pcl::KdTreeFLANN<pcl::PointXYZ>);
			tree_2->setInputCloud(cloud_cluster);
			std::vector<int> line_indices((*cloud_cluster).size()); 
			std::vector<float> line_dists((*cloud_cluster).size());
			tree_2->nearestKSearch(left_infinite, (*cloud_cluster).size(), line_indices, line_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
			
			point_set.push_back(cloud_cluster->points[line_indices[0]]);
			point_set.push_back(cloud_cluster->points[line_indices[-1]]);
			
		// PUBLISH ROS MESSAGES
			pcl::toROSMsg(closest, nearest_point);
			pcl::toROSMsg((*cloud_cluster), points_line);
			pcl::toROSMsg(reference_cloud, reference_point);
			pcl::toROSMsg(point_set, points_msg);
				
			reference_point.header.frame_id = scan_in->header.frame_id;
			nearest_point.header.frame_id = scan_in->header.frame_id;
			points_line.header.frame_id = scan_in->header.frame_id;
			points_msg.header.frame_id = scan_in->header.frame_id;
				
			this->pub_nearest_.publish(nearest_point);// current position		
			this->pub_ref_.publish(reference_point);
			this->pub_points_.publish(points_msg);
			this->pub_line_.publish(points_line);
		}
    }
    ~LineExtractRP(){
    }
    private:
        ros::NodeHandle nh_;
		ros::Subscriber sub_scan_;
        ros::Publisher pub_nearest_;
        ros::Publisher pub_ref_;
        ros::Publisher pub_points_;
		ros::Publisher pub_line_;
        laser_geometry::LaserProjection projector_;
		boost::recursive_mutex scope_mutex_;
};

class Command
{
public:
	Command():pnh_("~") {
		sub_joy_ = nh_.subscribe<sensor_msgs::Joy>("/joy", 5, &Command::handleJoyMode, this);
		sub_points_ = nh_.subscribe("/points_msg", 10, &Command::publishCmd,  this);
		sub_amcl_ = nh_.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &Command::handlePose, this);
		sub_goal_ = nh_.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &Command::setGoal, this);
		sub_obs_ = nh_.subscribe<sensor_msgs::PointCloud2>("/velodyne_points", 1, &Command::handleObstacle, this);
		pub_cmd_ = nh_.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
		pub_obs_ = nh_.advertise<sensor_msgs::PointCloud2> ("/obstacles", 10);


		};

	bool configure()
	{
        if (!params_.load(pnh_))
        {
            ROS_ERROR("Failed to load parameters");
            return false;
        }
		return true;
	}

	void handleJoyMode(const sensor_msgs::Joy::ConstPtr& joy_msg){

		//Button "B" : driving mode change -->   even: auto, odd: joy control
		if (joy_msg->buttons[1] == 1)
			driving_mode_ += 1; 

		if(this->driving_mode_ % 2 == 1){ // odd: joy control
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = joy_msg -> axes[1]*0.5;
			cmd_vel.angular.z = joy_msg -> axes[0]*0.5;
			this->pub_cmd_.publish(cmd_vel);
		}
	}

	void publishCmd(const sensor_msgs::PointCloud2 &cloud_msg) 
	{	
		boost::recursive_mutex::scoped_lock cmd_lock(scope_mutex_);
			
		if(this->driving_mode_ % 2 == 1) // joystick mode
			return;

		//else: autonomous driving
		geometry_msgs::Twist cmd_vel;
		float Kpy = params_.Kpy_param_; // rotation 
		float linear_vel = params_.linear_vel_;
		
		PointCloud2Ptr tmp_cloud(new PointCloud2);
		pcl_conversions::toPCL(cloud_msg, *tmp_cloud);
		PointCloudPtr data_cloud(new PointCloud); 
		pcl::fromPCLPointCloud2(*tmp_cloud, *data_cloud); 
		
	// 1. CALCULATE AISLE LINE LENGTH AND CENTER
		pcl::PointXYZ nearest_point = data_cloud->points[0];
		pcl::PointXYZ reference_point = data_cloud->points[1];
		if (!CHECK_LINE)
		{
			ROS_WARN("line was not detected yet");
			return;
		}
		float line_start = LINE_START;
		float line_end = LINE_END;
		float line_length = line_end - line_start;
		float left_boundary = line_start + (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);
		float right_boundary = line_end - (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);

	// 2. CONSIDER OBSTACLES
		
		// TODO: function for calculating the obstacles distance
/*
		float left_dist = 1000, right_dist = 1000, front_dist = 1000; // obstacles distance

		if(front_dist<params_.front_obstacle_dist_)
		{
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;
		}

		int closest_distance = left_dist < right_dist ? left_dist : -right_dist;
		float shift_position;

		if( abs(closest_distance) < params_.side_obstacle_dist_) 			
			shift_position = params_.obastalce_coefficient_ / closest_distance; 		 
		else 
			shift_position = 0.0;		

		cout<<"closest distance: "<<closest_distance<<endl;

		cout<<"shift position "<<shift_position<<endl;
*/
		float x_err_local = reference_point.x - nearest_point.x; 
		float y_err_local = reference_point.y - nearest_point.y;

		// Check boundary for safe driving
		if(left_boundary < line_start + 0.5*line_length && right_boundary > line_end - 0.5*line_length)
		{			
			if (nearest_point.y + shift_position_ > left_boundary && nearest_point.y + shift_position_ < right_boundary)   			
			{
				y_err_local += shift_position_;
				// cout<<"in boundary"<<endl;
			}
			else if (nearest_point.y + shift_position_ < left_boundary && shift_position_ < 0)
			{
				y_err_local = left_boundary - nearest_point.y;
				// cout<<"out boundary, stay in left boundary"<<endl;
			}
			else if (nearest_point.y + shift_position_ > right_boundary && shift_position_ > 0)
			{
				y_err_local = right_boundary - nearest_point.y;
				// cout<<"out boundary, stay in right boundary"<<endl;
			}
		} 
	// 3. CHECK GLOBAL GOAL
		bool has_arrived = checkArrival();
		float add_vel_local = 0;	

	// 4. PUBLISH COMMAND
		if (has_arrived || front_obstacle_)
		{
			cmd_vel.linear.x = 0.0;
			cmd_vel.angular.z = 0.0;		
		}	
		else
		{
			cmd_vel.linear.x = linear_vel;
			cmd_vel.angular.z = -Kpy*y_err_local;
		}
		this->pub_cmd_.publish(cmd_vel);	
	}

	bool checkArrival()
	{
		if (read_pose_ && has_goal_) // Drive until the robot arives to the goal
		{
			boost::recursive_mutex::scoped_lock pose_lock(scope_mutex_);
			float x_err_global, y_err_global, dist_err_global = 0.0; // global x, y, dist err
			x_err_global = clicked_point_.pose.position.x - amcl_pose_.pose.pose.position.x;
			y_err_global = clicked_point_.pose.position.y - amcl_pose_.pose.pose.position.y;
						
			dist_err_global = sqrt(x_err_global*x_err_global + y_err_global*y_err_global);
			
			return dist_err_global < 1.0;
		}

		return false;
	}

	void setGoal(const geometry_msgs::PoseStamped::ConstPtr& click_msg){
		clicked_point_.pose.position = click_msg->pose.position;
		has_goal_ =1;
	}   

	void handlePose(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
		amcl_pose_.pose.covariance = pose_msg->pose.covariance;
		amcl_pose_.pose.pose.position = pose_msg->pose.pose.position;
		amcl_pose_.pose.pose.orientation = pose_msg->pose.pose.orientation;
		this->read_pose_ = 1;
	}   
	

	void handleObstacle(const sensor_msgs::PointCloud2::ConstPtr& ros_pc)
	{
		boost::recursive_mutex::scoped_lock cmd_lock(scope_mutex_);
		
	    pcl::PCLPointCloud2 pcl_pc; // temporary PointCloud2 intermediary
		
	    pcl_conversions::toPCL(*ros_pc, pcl_pc);
	    // Convert point cloud to PCL native point cloud
	    PointCloud::Ptr input_ptr(new PointCloud());
	    pcl::fromPCLPointCloud2(pcl_pc, *input_ptr);

	    // Create output point cloud
	    PointCloud::Ptr output_ptr(new PointCloud());    	    
	    pcl::PassThrough<pcl::PointXYZ> pass;

	    pass.setInputCloud (input_ptr);         
	    pass.setFilterFieldName ("y");         
	    pass.setFilterLimits (-0.35, 0.35);    
	    //pass.setFilterLimitsNegative (true);  
	    pass.filter (*output_ptr);              

	  // Object for storing the plane model coefficients.
		pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
		pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
									
		pcl::SACSegmentation<pcl::PointXYZ> seg;
		seg.setOptimizeCoefficients (true);      
	  	seg.setInputCloud (output_ptr);                 
	  	seg.setModelType (pcl::SACMODEL_PLANE);    
	  	seg.setMethodType (pcl::SAC_RANSAC);      
	  	seg.setMaxIterations (1000);              
	  	seg.setDistanceThreshold (0.01);          
	  	seg.segment (*inliers, *coefficients);    

	   	pcl::ExtractIndices<pcl::PointXYZ> extract;
	   	extract.setInputCloud (output_ptr);
	   	extract.setIndices (inliers);
	   	extract.setNegative (true);//false
	   	extract.filter (*output_ptr);

																																																								
	    pass.setInputCloud(output_ptr);
	    pass.setFilterFieldName("z");           
	    pass.setFilterLimits(-0.27, 0.5);       
	    pass.filter(*output_ptr);             

	    pass.setInputCloud (output_ptr);      
	    pass.setFilterFieldName ("x");        
	    pass.setFilterLimits (0, 1);          
	    pass.filter (*output_ptr);           

		if(output_ptr->size() != 0){
		    pcl::StatisticalOutlierRemoval<pcl::PointXYZ> sor;
		    sor.setInputCloud (output_ptr);  
		    sor.setMeanK (50);               
		    sor.setStddevMulThresh (1.0);    
		    sor.filter (*output_ptr);        
		}

	    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);	    
	    if(output_ptr->size() != 0)
		{
		    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
		    tree->setInputCloud (output_ptr);  
		    std::vector<int> nn_indices(50);
		    std::vector<float> nn_dists(50);
		    pcl::PointXYZ origin(0, 0, 0);

		    tree->nearestKSearch(origin, 50, nn_indices, nn_dists);

			float min_negative = -999, min_positive = 999, movement = 0, min_positive_point =0, min_negative_point =0;
			int count_negative=0, count_positive = 0;
			
			float point_x, point_y, left_boundary, right_boundary, line_length;
			float line_start = LINE_START;
			float line_end = LINE_END;		
			line_length = line_end - line_start;

			for (int i = 0; i < nn_indices.size(); i++)
			{
				point_y = output_ptr->points[nn_indices[i]].y;
				point_x = output_ptr->points[nn_indices[i]].x;
				left_boundary = line_start + (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);
				right_boundary = line_end - (line_length * params_.boundary_percent_ + 0.5 * params_.robot_width_);
				
				
				if (abs(point_x) < params_.front_obstacle_dist_ && abs(left_boundary-point_y) < params_.robot_width_ && abs(right_boundary-point_y) < params_.robot_width_ )
				{
					cout<<"[FRONT OBSTACLES] Distance to obstacles(m): "<<point_y<<endl;
					front_obstacle_ = true;
					break;
				}
								
				if(point_y > 0) //obstacles in left side
				{			
					if(min_positive >point_y)
					{
						min_positive = point_y;
						min_positive_point = i;
					}
					count_positive += 1;
				}
				else //obstacles in right side
				{
					if(min_negative < point_y)
					{	
						min_negative = point_y;
						min_negative_point = i;
					}
					count_negative += 1;
				}
			}

				if(count_positive > count_negative) //obstacles in left side
				{
					shift_position_ = -sqrt(pow(output_ptr->points[nn_indices[min_positive_point]].x,2.0) + pow(output_ptr->points[nn_indices[min_positive_point]].y,2.0));
					std::cout<<"[LEFT OBSTACLES] Distance to obstacles(m): "<<shift_position_ <<std::endl;	
				}
				else	//obstacles in right side
				{	
					shift_position_ = sqrt(pow(output_ptr->points[nn_indices[min_negative_point]].x,2.0) + pow(output_ptr->points[nn_indices[min_negative_point]].y,2.0));
					std::cout<<"[RIGHT OBSTACLES] Distance to obstacles(m): "<<shift_position_ <<std::endl;
				}	
			
			front_obstacle_ = false;
	    }

	    //else
			//std::cout<<"[Go straight] No obstacles were detected" << std::endl;
	    

	// Convert data type PCL to ROS
	    sensor_msgs::PointCloud2 ros_output;
	    pcl::toPCLPointCloud2(*output_ptr, pcl_pc);
	    
	    pcl_conversions::fromPCL(pcl_pc, ros_output);

	    // Publish the data
	    pub_obs_.publish(ros_output);
	
	}

	~Command(){}

private:
	ros::NodeHandle nh_;
	ros::NodeHandle pnh_;
	ros::Subscriber sub_points_;
	ros::Subscriber sub_amcl_;
	ros::Subscriber sub_joy_;
	ros::Subscriber sub_goal_;
	ros::Subscriber sub_obs_;
	ros::Publisher pub_cmd_;
	ros::Publisher pub_obs_;

	geometry_msgs::PoseStamped clicked_point_;
	geometry_msgs::PoseWithCovarianceStamped amcl_pose_;

	Parameters params_; 

	boost::recursive_mutex scope_mutex_;
	int driving_mode_ = 0; // even: auto, odd: joy control
	bool read_pose_ = false;
	bool has_goal_ = false;
	bool has_arrived_ = false;     
	float shift_position_ = 0;
	bool front_obstacle_ = false;
	
	
};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "leo_driving_node");

	LineExtractRP LineExtractRP;
	Command Command;
	
	bool check = Command.configure();
	if (check)
		while(ros::ok()) 
    	    ros::spinOnce();
	
  	return 0;
}
