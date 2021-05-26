
#include "obs_avoid.h"


int main(int argc, char** argv)
{
  	ros::init(argc, argv, "velodyne_obstacle_node");
	VeloObs VeloObs;
	if (VeloObs.configure())
		while(ros::ok()) 
    	    ros::spinOnce();
  	return 0;
}
