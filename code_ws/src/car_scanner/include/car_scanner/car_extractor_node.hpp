/*
 * car_state_publisher.hpp
 *
 * This is the node to publish car states
 * 
 * Author: yubocheng@live.cn
 *
 * Copyright (c) 2019 Jimu
 * All rights reserved.
 *
 */

#ifndef CAR_EXTRACTOR_NODE
#define CAR_EXTRACTOR_NODE

#include <ros/ros.h>
#include "car_scanner/car_extractor.hpp"
#include <string>
using namespace car_scanner;

class CarExtractorROS: public CarExtractor{
public:
    void    LoadParam(ros::NodeHandle nh)           // load parameters from ROS
    {
        // TODO
        ros::param::get("/lidar_1/frame_id", left_lidar_frame_id);
        ros::param::get("/lidar_2/frame_id", right_lidar_frame_id);
        ros::param::get("/lidar_1/pose", transform_lidar_1);
        ros::param::get("/lidar_2/pose", transform_lidar_2);
        b_init = true;
    }
};

class CarExtractorNode{
    // Handlers
    ros::NodeHandle         nh;         // ROS node handler
    ros::Publisher          car_info_pub;
    CarInfo                 carInfoData;
    ros::Subscriber         wheel_info;

    CarExtractorROS         carExtractor;

public:
    bool Init()
    {
        // load parameters
        carExtractor.LoadParam(nh);

        // initialize publisher
        car_info_pub = nh.advertise<car_scanner::CarInfo>(std::string("car_info"), 1);

        // initialize subscriber
        wheel_info = nh.subscribe(std::string("wheel_info"), 1, &CarExtractorNode::OnWheelInfoRx, this);

        return true;
    }

    void Loop()
    {
        return;
    }

    void    OnWheelInfoRx(const car_scanner::WheelArray wheelInfo)
    {
        carExtractor.UpdateWheelInfo(wheelInfo);
        // TODO: get two, publish one
        if (carExtractor.GetCarInfo(carInfoData))
            car_info_pub.publish(carInfoData);
    }
};

#endif  // CAR_EXTRACTOR_NODE