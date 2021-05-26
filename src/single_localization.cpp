// Copyright 2021, Postech, Cocel, Control Team

#include "single_localization.h"

int main(int argc, char** argv)
{
ros::init(argc, argv, "single_localization_node");
SingleLocal SingleLocal;	
    while(ros::ok()) 
        ros::spinOnce();

return 0;
}

