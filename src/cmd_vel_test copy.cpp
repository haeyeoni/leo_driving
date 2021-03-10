// Copyright 2020, Postech, Cocel
//
// This code reads the sensor data from the RP Lidar to detect the center of the road.
// After finding the center position and the robot's direction, it sends the command to make the robot move to the center position.
// The code procedures are following

// 1. LineExtractRP class
// @Subscribes:
// 			* RPLidar sensor data ("/scan_rp")
// @Publishes: 
//			* Road line cluster that is possible range of robot. ("/cluster_line")
//			* Current robot's nearest point which can be used to infer the direction of the robot ("/nearest_point")
//			* Average coordinates of "/cluster_line" which can be used to infer the center of the rod ("/reference_point")
//			* Points data set of nearest point and reference point ("/points_msg")
// @Process
// 	1.1 Reads sensor data by subscribing "scan_rp" topic which is laserScan data type.
// 	1.2 Since RP Lidar scans 360 degree surrounding, the pointcloud range is croped to have forward area with 1 meter width. 
//  1.3 Using RANSAC algorithm, we extract the line segments from the point cloud.
//  1.4 Under asumption that the ground has two pits on eighter side of the road, the road line is extracted from the line clusters which are obtained at (1.3). 
//  1.5 After procedure, the class publish the multiple points data
//
// 2. Command class
// @Subscribes:
// 			* Points data processed by LineExtractRP class ("/points_msg")
// @Publishes: 
//			* Velocity commands ("/cmd_vel")
// @Process
// 	2.1 Reads the point data which is current direction point (points_msg[0]) and the road center point (points_msg[1]).
//  2.2 The command value is calculated accrording to the difference between the two points.
//  2.3 Publish the velocity command which has linear.x, angluar.z value.  


#include "cmd_vel_test.h"
#include <cmath>
#define samp_def 0.1
#define MAX_distance 1.0

// Argument linear_vel Kpy_param sonar_k mode boundary(default = 0.3)
// Argument 0.1 1.1 70 1 0.02

using namespace std;

class LineExtractRP
{
public:
    LineExtractRP() {
		this->subscriber = this->nh.subscribe("/up_scan", 10, &LineExtractRP::lineExtract, this);
		this->pub_line = this->nh.advertise<sensor_msgs::PointCloud2>("cluster_line", 10);
		this->pub_nearest = this->nh.advertise<sensor_msgs::PointCloud2> ("nearest_point", 10);
	 	this->pub_ref = this->nh.advertise<sensor_msgs::PointCloud2> ("reference_point", 10);
		this->pub_points = this->nh.advertise<sensor_msgs::PointCloud2> ("points_msg", 10);
    };
	bool configue()
	{

	}

    void lineExtract(const sensor_msgs::LaserScan::ConstPtr& scan_in) {

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
		if((*cloud_cluster).size()>0){
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
			float min = 1000;
			float max = 0;
			

			for (int i = 0; i < (*cloud_cluster).size(); i++) {
				filtered_cloud.push_back(cloud_cluster->points[i]);
				sum_x += cloud_cluster->points[i].x;
				sum_y += cloud_cluster->points[i].y;
				num_points ++;
				if (min > cloud_cluster->points[i].y) 
					min = cloud_cluster->points[i].y;
				if (max < cloud_cluster->points[i].y)
					max = cloud_cluster->points[i].y;
			}

			pthread_mutex_lock(&mutx_line);
			line_max = max;
			line_min = min;
			pthread_mutex_unlock(&mutx_line);


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
				
			reference_point.header.frame_id = "laser";
			nearest_point.header.frame_id = "laser";
			points_line.header.frame_id = "laser";
			points_msg.header.frame_id = "laser";
				
			this->pub_nearest.publish(nearest_point);// current position		
			this->pub_ref.publish(reference_point);
			this->pub_points.publish(points_msg);
			this->pub_line.publish(points_line);
			}
    }
    ~LineExtractRP(){
    }
    private:
        ros::NodeHandle nh;
		ros::Subscriber subscriber;
        ros::Publisher pub_nearest;
        ros::Publisher pub_ref;
        ros::Publisher pub_points;
		ros::Publisher pub_line;
        laser_geometry::LaserProjection projector_;
};

class Command
{
	public:
		Command() {
			this->sub_points = this->nh.subscribe("/points_msg", 10, &Command::PublishCmd,  this);
			this->sub_amcl = this->nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>("/amcl_pose", 10, &Command::CurrentPoint, this);
			this->pub_cmd = this->nh.advertise<geometry_msgs::Twist> ("/cmd_vel", 10);
			this->sub_goal = this->nh.subscribe<geometry_msgs::PoseStamped>("/move_base_simple/goal", 10, &Command::ReferencePoint, this);
			this->sub_joy = this->nh.subscribe<sensor_msgs::Joy>("/joy", 5, &Command::Joycallback, this);
		};

		bool configure()
		{
			
		}

	    void ReferencePoint(const geometry_msgs::PoseStamped::ConstPtr& click_msg){

			clicked_point_.pose.position = click_msg->pose.position;
			this->has_goal_ =1;
    	}   

		void Joycallback(const sensor_msgs::Joy::ConstPtr& joy_msg){
	
			//Button "B" : driving mode change -->   0: auto driving  // 1: manual driving
			if(this->driving_mode_checker == 0 && joy_msg->buttons[1] == 1){
				this->driving_mode += 1;
			}
			this->driving_mode_checker = joy_msg->buttons[1];

			//left button : extinguisher move change -->  up: move_up  // down: move_down
			move_num = joy_msg->axes[7];
			pthread_mutex_lock(&mutx_motor);
			extinguisher_move = move_num;
			pthread_mutex_unlock(&mutx_motor);
						
			//Button "X" : extinguisher jet -->  0: jet close  // 1: jet open
			if(this->extinguisher_jet_checker == 0 && joy_msg->buttons[2] == 1){
				pthread_mutex_lock(&mutx_motor);
				extinguisher_jet += 1;
				pthread_mutex_unlock(&mutx_motor);
			}
					this->extinguisher_jet_checker = joy_msg->buttons[2];		

			if(this->driving_mode%2 == 1){
				joy_cmd_vel(joy_msg);
			}
		}

		void joy_cmd_vel(const sensor_msgs::Joy::ConstPtr& joy_msg){
			//std::cout<<"manual_mode"<<std::endl;
			//Left axis : manual move command -->  up/down: linear velocity,   right/left: angluar velocity
			geometry_msgs::Twist cmd_vel;
			cmd_vel.linear.x = joy_msg -> axes[1]*0.5;
			cmd_vel.angular.z = joy_msg -> axes[0]*0.5;
			this->publisher.publish(cmd_vel);	
		}

	    void CurrentPoint(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& pose_msg){
	        amcl_pose_.pose.covariance = pose_msg->pose.covariance;
        	amcl_pose_.pose.pose.position = pose_msg->pose.pose.position;
        	amcl_pose_.pose.pose.orientation = pose_msg->pose.pose.orientation;
			this->read_pose_ = 1;
    	}   

		void PublishCmd(const sensor_msgs::PointCloud2 &cloud_msg) {
			
			PointCloud2Ptr tmp_cloud(new PointCloud2);
			pcl_conversions::toPCL(cloud_msg, *tmp_cloud);
			PointCloudPtr data_cloud(new PointCloud); 
			pcl::fromPCLPointCloud2(*tmp_cloud, *data_cloud); 
			
			pcl::PointXYZ nearest_point = data_cloud->points[0];
			pcl::PointXYZ reference_point = data_cloud->points[1];
			float line_start, line_end;
			
			pthread_mutex_lock(&mutx_line);
			line_start = line_min;
			line_end = line_max;
			pthread_mutex_unlock(&mutx_line);

			float line_length = line_end - line_start;
			float left_boundary = line_start + (line_length * boundary_percent + 0.5 * robot_width);
			float right_boundary = line_end - (line_length * boundary_percent + 0.5 * robot_width);
			
 			


			geometry_msgs::Twist cmd_vel;
			float xm_e = reference_point.x - nearest_point.x;
			float ym_e = reference_point.y - nearest_point.y;
			float shift_position;
			float Kpy = Kpy_param;
			float Left_distance=1000, Right_distance=1000, Front_distance=1000;
			float xg_e=0.0, yg_e =0.0,dis_err=0.0,X_vel=0.0;
			int stop_flag =0;
					
			if (this->checker1 == 1 && this->checker2 == 1)
			{
				float error;
				error = this->_amclPose.pose.pose.position.x - this->_clickedPoint.pose.position.x;
				std::cout <<"error: "<< error <<std::endl;
// error data

				xg_e = this->_clickedPoint.pose.position.x - this->_amclPose.pose.pose.position.x;
				yg_e = this->_clickedPoint.pose.position.y - this->_amclPose.pose.pose.position.y;
				std::cout<<"ref x: "<<this->_clickedPoint.pose.position.x<<std::endl;
				std::cout<<"cur x: "<<this->_amclPose.pose.pose.position.x<<std::endl;	
				std::cout<<"ref y: "<<this->_clickedPoint.pose.position.y<<std::endl;
				std::cout<<"cur y: "<<this->_amclPose.pose.pose.position.y<<std::endl;
			}
			
			float add_vel_local = 0;	
			pthread_mutex_lock(&mutx_sonar);
			Left_distance = L_dis;
			Right_distance = R_dis;
			Front_distance = F_dis;
			pthread_mutex_unlock(&mutx_sonar);
			cout << "Front sonar: " << Front_distance <<endl;
			
			
			//std::cout<< "L_Data: " <<Left_distance << " R_data: "<<Right_distance<<" F_Data: "<<Front_distance <<std::endl;
			int closest_distance = Left_distance < Right_distance ? Left_distance : -Right_distance;
			cout<<"left: "<< Left_distance <<" right: "<<Right_distance<<endl;				
			if(abs(closest_distance) <500 ) {			
			shift_position = sonar_K / closest_distance; 
			//cout<<"shifted: "<<shift_position<<endl;
			} else {
			shift_position = 0.0;		
			}
	// shift reference position - sonar
			if(left_boundary < line_start + 0.5*line_length && right_boundary > line_end - 0.5*line_length){
						
				if (nearest_point.y + shift_position > left_boundary && nearest_point.y + shift_position < right_boundary)   			
				{
					ym_e += shift_position;
					cout <<"in boundary"<<ym_e<<endl;
				}
				else if (nearest_point.y + shift_position < left_boundary && shift_position < 0)
				{
					ym_e = left_boundary - nearest_point.y;
					cout <<"out boundary, stay in left boundary "<<ym_e<<endl;
				}
				else if (nearest_point.y + shift_position > right_boundary && shift_position > 0)
				{
					ym_e = right_boundary - nearest_point.y;
					cout <<"out boundary, stay in right boundary "<<ym_e<<endl;
				}
			}
			else
			{
				cout<<"Boundary error"<<endl;
			}

			if(this->driving_mode%2 == 0){
				//std::cout<<"auto_mode"<<std::endl;
				if (linear_vel == 0.0)
				{
					cmd_vel.linear.x =0.0;
			  		cmd_vel.angular.z = 0.0;		
				}	
				else
				{
					dis_err =sqrt(xg_e*xg_e + yg_e*yg_e);
					X_vel= linear_vel/MAX_distance * dis_err;
					//std::cout<<"X Y:" <<xg_e<<", "<<yg_e <<std::endl;	
 					if( X_vel > linear_vel)
						X_vel = linear_vel;
					else if( X_vel < -linear_vel)
						X_vel = -linear_vel;					
					cmd_vel.linear.x = X_vel;
					cmd_vel.angular.z = -Kpy*ym_e;
					if(dis_err <1.0 || stop_flag)
					{
						cmd_vel.linear.x =0.0;
				  		cmd_vel.angular.z = 0.0;
					}
				}
				if(Mapping_dring)
				{
					cmd_vel.linear.x = linear_vel;
					cmd_vel.angular.z = -Kpy*ym_e;
				}
				if(Front_distance<300)
				{
					cmd_vel.linear.x = 0.0;
					cmd_vel.angular.z = 0.0;
				}
				this->driving_mode == 0;
				this->publisher.publish(cmd_vel);	
			}
		}
		void HandleFlag(const std_msgs::Bool::ConstPtr& arriveFlag)
	   	{
			_arriveFlag = arriveFlag->data;
	   	}

		~Command(){
		}

    private:
        ros::NodeHandle nh;
        ros::Subscriber sub_points;
        ros::Subscriber sub_amcl;
		ros::Subscriber sub_joy;
		ros::Subscriber sub_goal;
    	ros::Publisher pub_cmd;

        geometry_msgs::PoseStamped clicked_point_;
        geometry_msgs::PoseWithCovarianceStamped amcl_pose_;

		//driving_mode = 0 auto driving
		//driving_mode = 1 manual driving
		int driving_mode_checker = 0;
		int driving_mode = 0;
		int move_num =0;
		
		bool read_pose_ = false;
		bool has_goal_ = false;
		bool has_arived_ = false;
};

int main(int argc, char** argv)
{
  	ros::init(argc, argv, "leo_driving_node");

	LineExtractRP LineExtractRP;
	Command Command;
	
	bool check = LineExtractRP.configure() && Command.configure();

	if (check)
		while(ros::ok()) 
    	    ros::spinOnce();
	
  	return 0;
}
