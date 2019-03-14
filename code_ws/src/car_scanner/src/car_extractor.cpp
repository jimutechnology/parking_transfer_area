/*
 * car_extractor.cpp
 *
 * This is the class to extract car info from wheel info
 * 
 * Author: yubocheng@live.cn
 *
 * Copyright (c) 2019 Jimu
 * All rights reserved.
 *
 */

#include "car_scanner/car_extractor.hpp"
#include "obstacle_detector/utilities/point.h"
using namespace obstacle_detector;

void    CarExtractor::UpdateWheelInfo(const WheelInfo &wheelInfo)
{
    if (!b_init)
        return;
    // else b_init==TRUE

    /* Receive wheel info */
    if (wheelInfo.header.frame_id == left_lidar_frame_id)
    {
        b_available_left = true;
        if (wheelInfo.wheels[0].y > wheelInfo.wheels[1].y)
        {
            wheels[LEFT_FRONT] = wheelInfo.wheels[0];
            wheels[LEFT_REAR] = wheelInfo.wheels[1];
        }
        else // wheelInfo.wheels[0].y <= wheelInfo.wheels[1].y
        {
            wheels[LEFT_FRONT] = wheelInfo.wheels[1];
            wheels[LEFT_REAR] = wheelInfo.wheels[0];
        }
    }
    else if (wheelInfo.header.frame_id == right_lidar_frame_id)
    {
        b_available_right = true;
        if (wheelInfo.wheels[0].y > wheelInfo.wheels[1].y)
        {
            wheels[RIGHT_REAR] = wheelInfo.wheels[0];
            wheels[RIGHT_FRONT] = wheelInfo.wheels[1];
        }
        else // wheelInfo.wheels[0].y <= wheelInfo.wheels[1].y
        {
            wheels[RIGHT_REAR] = wheelInfo.wheels[1];
            wheels[RIGHT_FRONT] = wheelInfo.wheels[0];
        }
    }

    if(b_available_left && b_available_right)
    {
        b_available = true;
        CalculateWheelInfo();
    }
}

bool    CarExtractor::GetCarInfo(CarInfo &carInfo)
{
    if (b_available && b_init)
    {
        carInfo = this->carInfo;
        b_available = false;
        b_available_left = false;
        b_available_right = false;
        return true;
    }
    else
        return false;
}

void    CarExtractor::CalculateCarInfo()
{
    Point       first_points[NUM_WHEELS];   // feature points of wheels
    Point       second_ponits[NUM_WHEELS];
    Point       third_points[NUM_WHEELS];

    // -- Calculate LENGTH and WIDTH of wheels
    wheel_length_sample[NUM_WHEELS];    // distance(3P, 2P) of each wheel
    wheel_width_sample[NUM_WHEELS];     // distance(1P, 2P) of each wheel
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        wheel_length_sample[i] = distance(wheels[i].)
    }

    // -- Calculate orientation of the car

    // -- Calculate steering angle of front wheels

    // -- Calculate wheel distance
    // mid(left-back 2P & 3P'), mid(right_back 2P & 3P') across orientation
}
