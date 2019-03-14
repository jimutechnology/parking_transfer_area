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
using namespace car_scanner;

#define     NUM_WHEELS  4   // number of wheels on a car
/*
enum WheelIndex{
    LEFT_FRONT = 0,
    LEFT_REAR = 1,
    RIGHT_FRONT = 2,
    RIGHT_REAR = 3
};
*/
#define     LEFT_FRONT  0
#define     LEFT_REAR   1
#define     RIGHT_FRONT 2
#define     RIGHT_REAR  3

class CarExtractor{
public:
    CarExtractor(): b_available(false),
                    b_available_left(false),
                    b_available_right(false),
                    b_init(false)
                    {}

protected:
    bool    b_available_left;               // whether car info or left/right wheel info available
    bool    b_available_right;
    bool    b_available;
    bool    b_init;                         // whether object initialized


public:
    void    LoadParam();                    // load parameters -- BE SURE TO SET b_init
    void    UpdateWheelInfo(const WheelArray &wheelInfo);    // receive wheel info and update
    bool    GetCarInfo(CarInfo &carInfo);   // Get car-info from this object

private:
    void    CalculateCarInfo();             // calculate car info based on wheel info

private:
    Wheel       wheels[NUM_WHEELS];         // all wheels' info

    float       wheel_length;               // wheel length for all, [m]
    float       wheel_width;                // wheel width for all, [m]
    float       wheelbase_length;           // wheel base length, [m]
    float       wheel_distance;             // wheel distance, [m]

};

#endif  // CAR_EXTRACTOR_H__