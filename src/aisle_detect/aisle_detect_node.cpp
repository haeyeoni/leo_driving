
#include <tunnel_driving/aisle_detect.hpp>
#include <pluginlib/class_list_macros.hpp>


namespace tunnel_driving
{

AisleDetectNode::AisleDetectNode()
{
    aisle_detector_.set_config();
}
void set_config()
{
    ros::NodeHandle &nh = getMTNodeHandle();
    ros::NodeHandle &private_nh = getMTPrivateNodeHandle();

    private_nh.param("line_thresh", config_.line_thresh_, 0.5);
    private_nh.param("line_thresh", config_.aisle_width_, 0.6);

}
void AisleDetectNode::onInit()
{

    sub_scan_ = nh_.subscribe("/up_scan", 10, &AisleDetectNode::scanCallback, this);
    pub_line_ = nh_.advertise<sensor_msgs::PointCloud2>("cluster_line", 10);
    pub_points_ = nh_.advertise<sensor_msgs::PointCloud2> ("points_msg", 10);
    // pub_nearest_ = nh_.advertise<sensor_msgs::PointCloud2> ("nearest_point", 10);
    // pub_ref_ = nh_.advertise<sensor_msgs::PointCloud2> ("reference_point", 10);
}

void AisleDetectNode::scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
{
    // 1. Conversion from ROS msgs -> pcl pointcloud
    sensor_msgs::PointCloud2 cloud_temp; // <- temporary point cloud to temporaly save the input point cloud     
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>); // <- data cloud
    pcl::PCLPointCloud2::Ptr cloud2(new pcl::PCLPointCloud2); // <- data cloud2 (converted from the cloud)
    projector_.projectLaser(*scan_msg, cloud_temp);
    pcl_conversions::toPCL(cloud_temp, *cloud2); 
    pcl::fromPCLPointCloud2(*cloud2, *cloud); 

    // 2. Crop Point Cloud
    pcl::ConditionAnd<pcl::PointXYZ>::Ptr range_condition(new pcl::ConditionAnd<pcl::PointXYZ>());
    range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::GT, -config_.aisle_width_)));
    range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("y", pcl::ComparisonOps::LT, config_.aisle_width_)));
    range_condition->addComparison (pcl::FieldComparison<pcl::PointXYZ>::ConstPtr (new pcl::FieldComparison<pcl::PointXYZ>("x", pcl::ComparisonOps::LT, 0.0)));
    pcl::ConditionalRemoval<pcl::PointXYZ> condrem;
    condrem.setInputCloud(cloud);
    condrem.setCondition(range_condition);
    condrem.setKeepOrganized(true);
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_inrange(new pcl::PointCloud<pcl::PointXYZ>); // <- cropped cloud
    condrem.filter(*cloud_inrange);
    
    // 3. Extract Line (RANSAC)
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients());
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices());
    pcl::SACSegmentation<pcl::PointXYZ> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_LINE); // <- extract model setting
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.maxIterations(1000);
    seg.setDistanceThreshold(config_.line_thresh_); // <- threshold (line width) // 0.5
    seg.setInputCloud(cloud_inrange); 
    seg.segment(*inliers, *coefficients);
    if (inliers->indices.size () == 0)
    {
        std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
        break;
    }
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_line (new pcl::PointCloud<pcl::PointXYZ>);
    pcl::ExtractIndices<pcl::PointXYZ> extract;
    extract.setInputCloud(cloud_inrange);
    extract.setIndices(inliers);
    extract.setNegative(false); //<- if true, it returns point cloud except the line.
    extract.filter(*cloud_line);

    // 4. Select Closest Line (: first cluster)
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree_cluster(new pcl::search::KdTree<pcl::PointXYZ>);
    tree_cluster->setInputCloud(cloud_line);
    std::vector<pcl::PointIndices> cluster_indices;
    pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
    ec.setInputCloud(cloud_line);
    ec.setClusterTolerance(0.05); // <- If the two points have distance bigger than this tolerance, then points go to different clusters. 
    ec.setMinClusterSize(30); 
    ec.setMaxClusterSize(800);;
    ec.setSearchMethod(tree_cluster);
    ec.extract(cluster_indices);		

        // clustering the line
    std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin ();
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_cluster (new pcl::PointCloud<pcl::PointXYZ>);
    for (std::vector<pcl::PointIndices>::const_iterator it = cluster_indices.begin (); it != cluster_indices.end (); ++it)
    {
        for (std::vector<int>::const_iterator pit = it->indices.begin (); pit != it->indices.end (); ++pit)
        {		
            cloud_cluster->points.push_back (cloud_line->points[*pit]);
        }
        cloud_cluster->width = cloud_cluster->points.size ();
        cloud_cluster->height = 1;
        cloud_cluster->is_dense = true;
        break;
    }

    // 5. Find the nearest point from the origin 
    pcl::PointCloud<pcl::PointXYZ> point_set;
    if((*cloud_cluster).size() == 0)
    {
        ROS_WARN("Not enough points!");
        return;
    } 

    pcl::PointXYZ origin(0, 0, 0);	
    pcl::KdTree<pcl::PointXYZ>::Ptr tree_(new pcl::KdTreeFLANN<pcl::PointXYZ>);
    tree_->setInputCloud(cloud_cluster);
    std::vector<int> nn_indices(1); 
    std::vector<float> nn_dists(1);
    tree_->nearestKSearch(origin, 1, nn_indices, nn_dists); //<- finds the most closest sing point: save points index to "nn_indices", and distance to "nn_dists"
    point_set.push_back(cloud_cluster->points[nn_indices[0]]); // point_set[0]

    // 6. Find the reference point (center points of aisle) && find line start and end
    float line_start = 0;
    float line_end = 10000;
    float sum_x = 0;
    float num_points = 0;

    for (int i = 0; i < cloud_cluster->size(); i++) 
    {
        sum_x += cloud_cluster->points[i].x;
        num_points ++;
        if (line_end > cloud_cluster->points[i].y) 
            line_end = cloud_cluster->points[i].y;
        if (line_start < cloud_cluster->points[i].y)
            line_start = cloud_cluster->points[i].y;
    }
    
    pcl::PointXYZ reference (sum_x / (float)num_points, (line_start + line_end)/2, 0);
    point_set.push_back(reference); // point_set[1]
 
    pcl::PointXYZ line_start(sum_x / (float)num_points, line_start, 0); 
    pcl::PointXYZ line_end(sum_x / (float)num_points, line_end, 0);
    
    point_set.push_back(line_start); // point_set[2]
    point_set.push_back(line_end); // point_set[3]
    
    // 7. Publish Points msgs

    // Messages to be published  
    sensor_msgs::PointCloud2 points_msg;
    sensor_msgs::PointCloud2 points_line;

    pcl::toROSMsg((*cloud_cluster), points_line);
    pcl::toROSMsg(point_set, points_msg);
        
    points_line.header.frame_id = scan_in->header.frame_id;
    points_msg.header.frame_id = scan_in->header.frame_id;

    this->pub_points_.publish(points_msg);
    this->pub_line_.publish(points_line);	
}

} // namespace tunnel_driving
PLUGINLIB_EXPORT_CLASS(AisleDetectNode, nodelet::Nodelet)