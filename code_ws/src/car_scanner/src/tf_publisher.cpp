/*
 * tf_publisher.cpp
 *
 * This is the node to publish (static) tf information.
 *
 * Author : yubocheng@jimu.ai
 *
 * Copyright (c) 2020 Jimu
 * All rights reserved.
 *
 */

#include <ros/ros.h>
#include <vector>
#include <tf/transform_broadcaster.h>
#include <string>
#include "math.h"

using namespace std;

#define TF_RATE     1
#define FRAME_TRANSFER_AREA     "transfer"
int main(int argc, char **argv)
{
    ros::init(argc, argv, "tf_publisher");

    vector<double>  pose_lidar_1;   // pose [x, y, yaw] of lidar 1
    vector<double>  pose_lidar_2;
    string          frame_lidar_1;  // frame ID of lidar 1
    string          frame_lidar_2;
    bool            b_lidar_upsidedown; // for both lidars
    double          angle_roll_lidars;
    tf::TransformBroadcaster br;
    tf::Transform transform_lidar_1;
    tf::Transform transform_lidar_2;
    ros::Rate node_rate(TF_RATE);
    tf::Quaternion q;
    ros::NodeHandle     nh;

    ros::param::get("/lidar_1/pose", pose_lidar_1);
    ros::param::get("/lidar_2/pose", pose_lidar_2);
    ros::param::get("/lidar_1/frame_id", frame_lidar_1);
    ros::param::get("/lidar_2/frame_id", frame_lidar_2);
    ros::param::get("/obstacle_extractor/b_lidar_upsidedown", b_lidar_upsidedown);
    if (b_lidar_upsidedown)
        angle_roll_lidars = M_PI;
    else
        angle_roll_lidars = 0.0;

    transform_lidar_1.setOrigin(tf::Vector3(pose_lidar_1[0], pose_lidar_1[1], 0.0));
    q.setRPY(0, 0, pose_lidar_1[2]);
    transform_lidar_1.setRotation(q);    
    transform_lidar_2.setOrigin(tf::Vector3(pose_lidar_2[0], pose_lidar_2[1], 0.0));
    q.setRPY(0, 0, pose_lidar_2[2]);
    transform_lidar_2.setRotation(q);

    while (ros::ok())
    {
        br.sendTransform(tf::StampedTransform(transform_lidar_1, ros::Time::now(), FRAME_TRANSFER_AREA, frame_lidar_1));
        br.sendTransform(tf::StampedTransform(transform_lidar_2, ros::Time::now(), FRAME_TRANSFER_AREA, frame_lidar_2));
        ros::spinOnce();
        node_rate.sleep();
    }
}
