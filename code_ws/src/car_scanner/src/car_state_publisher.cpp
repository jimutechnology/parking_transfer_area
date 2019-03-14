/*
 * car_state_analysis_node.cpp
 *
 * This is the node to analyze and publish car states.
 *
 * Author: yubocheng@live.cn
 *
 * Copyright (c) 2019 Jimu
 * All rights reserved.
 *
 */
#include <car_scanner/car_state_publisher.hpp>

int main(int argc, char **argv)
{
    /* create node name "wheel_odometry" */
    ros::init(argc, argv, "car_state_publisher");

    CarStatePublisher carStatePublisher;

    if (!carStatePublisher.Init())
    	return -1;
    
    while (ros::ok())
    {
    	ros::spinOnce();
    	carStatePublisher.Loop();
    }
    
    return 0;
}
