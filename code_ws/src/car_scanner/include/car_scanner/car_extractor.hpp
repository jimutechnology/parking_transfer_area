/*
 * car_extractor.hpp
 *
 * This is the class to extract car info from wheel info
 * 
 * Author: yubocheng@live.cn
 *
 * Copyright (c) 2019 Jimu
 * All rights reserved.
 *
 */

#ifndef CAR_EXTRACTOR_H__
#define CAR_EXTRACTOR_H__

#include "car_scanner/CarInfo.h"
#include "car_scanner/WheelArray.h"
#include "car_scanner/Wheel.h"
#include <string>
#include <vector>
using namespace car_scanner;
using namespace std;

enum WheelIndex{
    LEFT_FRONT = 0,
    LEFT_REAR = 1,
    RIGHT_FRONT = 2,
    RIGHT_REAR = 3, 
    NUM_WHEELS = 4      // number of wheels
};

class CarExtractor{
public:
    CarExtractor(): b_available(false),
                    b_available_left(false),
                    b_available_right(false),
                    b_init(false),
                    b_debug_on(false),
                    wheel_length_option(MAX)
                    {}

protected:
    bool    b_available_left;               // whether car info or left/right wheel info available
    bool    b_available_right;
    bool    b_available;
    bool    b_init;                         // whether object initialized
    bool    b_debug_on;                     // switch on debug output

    string  left_lidar_frame_id;            // frame id (string: e.g. lidar_1) of lidars
    string  right_lidar_frame_id;

    vector<double>  transform_lidar_1;      // transformation from transfer frame to lidar frame      
    vector<double>  transform_lidar_2;

    enum    WheelLengthOption {
        MAX,
        AVERAGE,
        MEDIAN
    };
    WheelLengthOption wheel_length_option;
    string            wheel_length_option_str;

public:
    void    LoadParam();                    // load parameters 
                                            // -- BE SURE TO SET b_init, left_lidar_frame_id, right_lidar_frame_id
    void    UpdateWheelInfo(const WheelArray &wheelInfo);    // receive wheel info and update
    bool    GetCarInfo(CarInfo &carInfo);   // Get car-info from this object

private:
    void    CalculateCarInfo();             // calculate car info based on wheel info

private:
    Wheel       wheels[NUM_WHEELS];         // all wheels' info
    CarInfo     carInfo;                    // car info


};

#endif  // CAR_EXTRACTOR_H__