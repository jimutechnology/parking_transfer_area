/*
 * car_extractor_node.cpp
 *
 * This is the node to extract car states.
 *
 * Author: yubocheng@live.cn
 *
 * Copyright (c) 2019 Jimu
 * All rights reserved.
 *
 */
#include <car_scanner/car_extractor_node.hpp>

int main(int argc, char **argv)
{
    /* create node name "wheel_odometry" */
    ros::init(argc, argv, "car_extractor_node");

    CarExtractorNode carExtractorNode;

    if (!carExtractorNode.Init())
        return -1;
    
    while (ros::ok())
    {
        ros::spinOnce();
        carExtractorNode.Loop();
    }
    
    return 0;
}
