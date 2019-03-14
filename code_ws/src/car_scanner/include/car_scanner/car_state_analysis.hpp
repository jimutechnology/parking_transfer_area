/*
 * car_state_analysis.hpp
 *
 * This is the class to analyze the state of the car.
 * 
 * Author: yubocheng@live.cn
 *
 * Input: 
 * 	- Car info, including pose of the car, steering angle of front wheels, etc.
 * Configuration:
 *  - Limits of states
 * Output:
 *  - Car states, including whether offset of pose is too large, steerin angle is too large, etc.
 * Copyright (c) 2019 Jimu
 * All rights reserved.
 *
 */

#ifndef CAR_STATE_ANALYSIS
#define CAR_STATE_ANALYSIS

#include "car_scanner/CarInfo.h"
#include "car_scanner/CarState.h"
using namespace car_scanner;

class CarStateAnalysis{
public:
    CarStateAnalysis(): b_available(false),
	                    b_init(false) 
	                    {}

protected:                        
	bool 	b_available;							// whether states available
	bool 	b_init;									// whether class (configuration) is initialized
	CarState 	carState;							// states of the car
	CarInfo 	carInfo;							// info of the car
	// -- spec limits for the car
	float 	max_wheels_distance;					// max wheel distance, [m]	
	float 	min_wheels_distance;					// min wheel distance, [m]
	float 	max_wheelbase; 							// max wheelbase, [m]
	float 	min_wheelbase; 							// min wheelbase, [m]
	// -- limits for the pose of the car
	float 	max_car_yaw; 							// max yaw of the car, [rad]
	float 	min_car_yaw;							// min yaw of the car, [rad]
	float 	max_front_steering_angle; 				// max steering angle of front wheels of the car, [rad]
	float 	min_front_steering_angle; 				// min steering angle of front wheels of the car, [rad]
	float 	max_car_x; 								// max X position of the car, [m]
	float 	min_car_x;	 							// min X position of the car, [m]
	float 	max_car_y; 								// max Y position of the car, [m]
	float 	min_car_y; 								// min Y position of the car, [m]

public:
	void 	LoadParam();							// load parameters -- MUST SET b_init
	void	UpdateCar(const CarInfo &carInfo);		// Receive car-info and update car-state
	bool 	GetCarState(CarState &carState);		// Get car-state from the object
		
};

#endif  // CAR_STATE_ANALYSIS