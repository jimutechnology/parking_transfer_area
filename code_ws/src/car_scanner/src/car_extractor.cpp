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
#include "obstacle_detector/utilities/math_utilities.h"
#include <iostream>
using namespace std;
using namespace obstacle_detector;

void    CarExtractor::UpdateWheelInfo(const WheelArray &wheelInfo)
{
    if (!b_init)
        return;
    // else b_init==TRUE

    /* Receive wheel info */
    // w.r.t. lidar frame
    if (wheelInfo.header.frame_id == left_lidar_frame_id)
    {
        b_available_left = true;
        if (wheelInfo.wheels[0].second_point.y > wheelInfo.wheels[1].second_point.y)
        {
            wheels[LEFT_FRONT] = wheelInfo.wheels[0];
            wheels[LEFT_REAR] = wheelInfo.wheels[1];
        }
        else // wheelInfo.wheels[0].second_point.y <= wheelInfo.wheels[1].second_point.y
        {
            wheels[LEFT_FRONT] = wheelInfo.wheels[1];
            wheels[LEFT_REAR] = wheelInfo.wheels[0];
        }
    }
    else if (wheelInfo.header.frame_id == right_lidar_frame_id)
    {
        b_available_right = true;
        if (wheelInfo.wheels[0].second_point.y > wheelInfo.wheels[1].second_point.y)
        {
            wheels[RIGHT_REAR] = wheelInfo.wheels[0];
            wheels[RIGHT_FRONT] = wheelInfo.wheels[1];
        }
        else // wheelInfo.wheels[0].second_point.y <= wheelInfo.wheels[1].second_point.y
        {
            wheels[RIGHT_REAR] = wheelInfo.wheels[1];
            wheels[RIGHT_FRONT] = wheelInfo.wheels[0];
        }
    }

    if(b_available_left || b_available_right)
    {
        b_available = true;
        CalculateCarInfo();
    }
}

bool    CarExtractor::GetCarInfo(CarInfo &carInfo)
{
    if (b_debug_on)
    {
        cout << b_available << b_available_left << b_available_right << endl;
    }

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
    Point       second_points[NUM_WHEELS];
    Point       third_points[NUM_WHEELS];
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        first_points[i].x = wheels[i].first_point.x;
        first_points[i].y = wheels[i].first_point.y;
        second_points[i].x = wheels[i].second_point.x;
        second_points[i].y = wheels[i].second_point.y;
        third_points[i].x = wheels[i].third_point.x;
        third_points[i].y = wheels[i].third_point.y;
    }
    first_points[LEFT_FRONT] = transformPoint(first_points[LEFT_FRONT], transform_lidar_1[0], transform_lidar_1[1], transform_lidar_1[2]);
    second_points[LEFT_FRONT] = transformPoint(second_points[LEFT_FRONT], transform_lidar_1[0], transform_lidar_1[1], transform_lidar_1[2]);
    third_points[LEFT_FRONT] = transformPoint(third_points[LEFT_FRONT], transform_lidar_1[0], transform_lidar_1[1], transform_lidar_1[2]);
    first_points[LEFT_REAR] = transformPoint(first_points[LEFT_REAR], transform_lidar_1[0], transform_lidar_1[1], transform_lidar_1[2]);
    second_points[LEFT_REAR] = transformPoint(second_points[LEFT_REAR], transform_lidar_1[0], transform_lidar_1[1], transform_lidar_1[2]);
    third_points[LEFT_REAR] = transformPoint(third_points[LEFT_REAR], transform_lidar_1[0], transform_lidar_1[1], transform_lidar_1[2]);
    first_points[RIGHT_FRONT] = transformPoint(first_points[RIGHT_FRONT], transform_lidar_2[0], transform_lidar_2[1], transform_lidar_2[2]);
    second_points[RIGHT_FRONT] = transformPoint(second_points[RIGHT_FRONT], transform_lidar_2[0], transform_lidar_2[1], transform_lidar_2[2]);
    third_points[RIGHT_FRONT] = transformPoint(third_points[RIGHT_FRONT], transform_lidar_2[0], transform_lidar_2[1], transform_lidar_2[2]);
    first_points[RIGHT_REAR] = transformPoint(first_points[RIGHT_REAR], transform_lidar_2[0], transform_lidar_2[1], transform_lidar_2[2]);
    second_points[RIGHT_REAR] = transformPoint(second_points[RIGHT_REAR], transform_lidar_2[0], transform_lidar_2[1], transform_lidar_2[2]);
    third_points[RIGHT_REAR] = transformPoint(third_points[RIGHT_REAR], transform_lidar_2[0], transform_lidar_2[1], transform_lidar_2[2]);
    
    if (b_debug_on)
    {
        for (int i = 0; i < NUM_WHEELS; i++)
        {
            cout << "W" << i << "= [" << first_points[i].x << ", " << first_points[i].y << "; "
                             <<  " "  << second_points[i].x << ", " << second_points[i].y << "; "
                             <<  " "  << third_points[i].x << ", " << third_points[i].y << "]; "
                             << endl;
        }
    }

    // -- Calculate LENGTH and WIDTH of wheels
    float   wheel_length_sample[NUM_WHEELS];    // distance(3P, 2P) of each wheel
    float   wheel_width_sample[NUM_WHEELS];     // distance(1P, 2P) of each wheel
    float   wheel_length_average = 0;
    float   wheel_width_average = 0;
    double  wheel_length_max = 0;
    double  wheel_length;
    for (int i = 0; i < NUM_WHEELS; i++)
    {
        wheel_length_sample[i] = third_points[i].distance(second_points[i]);
        wheel_width_sample[i] = first_points[i].distance(second_points[i]);
        wheel_length_average += wheel_length_sample[i];
        wheel_width_average += wheel_width_sample[i];
        if (wheel_length_max < wheel_length_sample[i])
            wheel_length_max = wheel_length_sample[i];
    }
    wheel_length_average /= NUM_WHEELS;
    wheel_width_average /= NUM_WHEELS;

    // Choose the right option for wheel length estimation
    if (wheel_length_option == MAX)
        wheel_length = wheel_length_max;
    else if (wheel_length_option == AVERAGE)
        wheel_length = wheel_length_average;
    else // by default
        wheel_length = wheel_length_max;

    // -- Calculate orientation of the car
    float       angle_wheels[NUM_WHEELS];
    angle_wheels[LEFT_FRONT] = second_points[LEFT_FRONT].orientation(third_points[LEFT_FRONT]);
    angle_wheels[LEFT_REAR] = third_points[LEFT_REAR].orientation(second_points[LEFT_REAR]);
    angle_wheels[RIGHT_FRONT] = second_points[RIGHT_FRONT].orientation(third_points[RIGHT_FRONT]);
    angle_wheels[RIGHT_REAR] = third_points[RIGHT_REAR].orientation(second_points[RIGHT_REAR]);
    carInfo.yaw = (angle_wheels[LEFT_REAR] + angle_wheels[RIGHT_REAR]) / 2;

    // -- Calculate steering angle of front wheels
    if (b_debug_on)
    {
        for (int i = 0; i < NUM_WHEELS; i++)
            cout << angle_wheels[i] << ", " ;
        cout << "angle wheels" << endl;
    }
    carInfo.steering = (angle_wheels[LEFT_FRONT] + angle_wheels[RIGHT_FRONT]) / 2 - carInfo.yaw;

    // -- Calculate wheel distance
    carInfo.length_outer_wheel_distance = second_points[LEFT_REAR].distance(second_points[RIGHT_REAR]);
    carInfo.length_inner_wheel_distance = carInfo.length_outer_wheel_distance - wheel_width_average * 2;

    // -- Calculate wheel base
    // TODO: not exactly following the design
    // Point       mid_front_wheels = ((third_points[LEFT_FRONT] + second_points[LEFT_FRONT]) / 2
    //                               + (third_points[RIGHT_FRONT] + second_points[RIGHT_FRONT]) / 2) / 2;
    // Point       mid_rear_wheels = ((third_points[LEFT_REAR] + second_points[LEFT_REAR]) / 2
    //                              + (third_points[RIGHT_REAR] + second_points[RIGHT_REAR]) / 2 ) / 2;
    Point       mid_front_wheels = (second_points[LEFT_FRONT] + second_points[RIGHT_FRONT]) / 2;
    Point       mid_rear_wheels = (second_points[LEFT_REAR] + second_points[RIGHT_REAR]) / 2;
    if (b_debug_on)
    {
        cout << "{" << mid_front_wheels.x << ", " << mid_front_wheels.y << "} " 
             << "{" << mid_rear_wheels.x << ", " << mid_rear_wheels.y << "} " 
             << "--" << wheel_length << endl;
    }
    carInfo.length_wheelbase = mid_front_wheels.distance(mid_rear_wheels) + wheel_length; // This is not real mid_front_wheels -- mid_rear_wheels

    // -- Calculate center point of car (of wheels, to be exact)
    Point       mid_point_car = (mid_front_wheels + mid_rear_wheels) / 2;
    carInfo.X = mid_point_car.x;
    carInfo.Y = mid_point_car.y;

    // -- Calculate access point of car (middle point of the front edge of front wheels)
    Point       access_point = mid_front_wheels 
                             + Point(wheel_length * cos(carInfo.yaw), wheel_length * sin(carInfo.yaw));
    carInfo.aX = access_point.x;
    carInfo.aY = access_point.y;
    carInfo.aYaw = carInfo.yaw;                         

    if (b_debug_on)
    {
        cout << "Car Length Info: " << carInfo.length_wheelbase << ", " << carInfo.length_outer_wheel_distance << ", " << carInfo.length_inner_wheel_distance << endl;
    }
    return;
}
