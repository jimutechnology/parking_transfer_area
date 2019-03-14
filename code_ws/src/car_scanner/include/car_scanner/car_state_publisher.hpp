/*
 * car_state_publisher.hpp
 *
 * This is the class to analyze and publish the state of the car.
 * 
 * Author: yubocheng@live.cn
 *
 * Copyright (c) 2019 Jimu
 * All rights reserved.
 *
 */

#ifndef CAR_STATE_PUBLISHER
#define CAR_STATE_PUBLISHER

#include <ros/ros.h>
#include "car_scanner/CarInfo.h"
#include "car_scanner/CarState.h"
#include "car_state_analysis.hpp"
#include <string>
using namespace car_scanner;
using namespace std;

class CarStateAnalysisROS: public CarStateAnalysis{
public:
	void 	LoadParam(ros::NodeHandle nh)			// load parameters from ROS
	{
		ros::param::get("car_pose_limit/min_car_yaw", min_car_yaw);
		ros::param::get("car_pose_limit/max_car_yaw", max_car_yaw);
		ros::param::get("car_pose_limit/min_car_x", min_car_x);
		ros::param::get("car_pose_limit/max_car_x", max_car_x);
		ros::param::get("car_pose_limit/min_car_y", min_car_y);
		ros::param::get("car_pose_limit/max_car_y", max_car_y);
		ros::param::get("car_pose_limit/min_front_steering_angle", min_front_steering_angle);
		ros::param::get("car_pose_limit/max_front_steering_angle", max_front_steering_angle);
        ros::param::get("car_spec/max_wheels_distance", max_wheels_distance);
        ros::param::get("car_spec/min_wheels_distance", min_wheels_distance);
        ros::param::get("car_spec/max_wheelbase", max_wheelbase);
        ros::param::get("car_spec/min_wheelbase", min_wheelbase);
		
		b_init = true;
	}
	
};

class CarStatePublisher{
	// Handlers
	ros::NodeHandle 	nh;				// ROS node handler
	ros::Publisher 		car_state_pub; 	
	car_scanner::CarState 	carStateData;
	ros::Subscriber 	car_info_sub;

	CarStateAnalysisROS carStateAnalyzer;

public:
	bool Init()
	{
		// load parameters
		carStateAnalyzer.LoadParam(nh);

		// initialize publisher
		car_state_pub = nh.advertise<car_scanner::CarState>(string("car_state"), 1);

		// initialize subscriber
		car_info_sub = nh.subscribe(string("car_info"), 1, &CarStatePublisher::OnCarInfoRx, this);

		return true;
	}

	void Loop()
	{
		return;
	}

	void 	OnCarInfoRx(const car_scanner::CarInfo carInfo)
	{
		carStateAnalyzer.UpdateCar(carInfo);
		carStateAnalyzer.GetCarState(carStateData);
		car_state_pub.publish(carStateData);
	}
};

#endif  // CAR_STATE_PUBLISHER