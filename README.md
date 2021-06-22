# Autonomous Tunnel Driving package

This package is for running mobile robot under tunnel environment.

### Used Sensors are
* 45-degree tilted 2D Lidar: for aisle-follow driving 
* 3D LiDAR on mobile robot: for detecting obstacles, mapping, localization

### Consisting Nodes are
* #### Aisle detect Node
    * Subscribe
        * /rp/scan (sensor_msgs/LaserScan)
    * Publish: 
        * /cluster_line (sensor_msgs/PointCloud2): detected aisle
        * /aisle_points (sensor_msgs/PointCloud2): robot's direction on line, line center, line start, line end
* #### Obstacles Node
    * Subscribe
        * /velodyne_points (sensor_msgs/PointCloud2)
    * Publish: 
        * /cropped_obs (sensor_msgs/PointCloud2): detected obstacles within the aisle zone
        * /obs_dists (std_msgs/Float32MultiArray): Right and Left obstacles's distances 
* #### Localization Node
    * Subscribe
        * /amcl_pose (geometry_msgs/PoseWithCovarianceStamped): amcl_pose topic can be published by running [amcl package](http://wiki.ros.org/amcl) simultaneously.
        * /move_base_simple/goal (geometry_msgs/PoseStamped): The target position can be set with __2D Nav Goal__ of Rviz.
    * Publish: 
        * /localization_data (std_msgs/Float32MultiArray): [0]-global dist error, [1]-global angle error, [2]-arrival flag, [3]-rotating flag
* #### Cmd Puslish Node
    * Subscribe (data from other nodes)
        * /joystick (sensor_msgs::Joy): amcl_pose topic can be published by running [amcl package](http://wiki.ros.org/amcl) simultaneously.
        * /obs_dists (std_msgs/Float32MultiArray)
        * /aisle_points (sensor_msgs/PointCloud2)
        * /localization_data (std_msgs/Float32MultiArray)
        * /lidar_driving (std_msgs/Bool) : Should be puslished manually to execute the /cmd_vel publishing function.
    * Publish: 
        * /cmd_vel (geometry_msgs/Twist): Final cmd_vel combining information from multiple nodes

### To run the package (after building the code)
```bash
(terminal1) roslaunch auto_driving run_nodelet.launch 
(termianl2) rostopic pub -r 10 /lidar_driving std_msgs/Bool True

```
