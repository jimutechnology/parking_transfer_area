#include <car_scanner/car_state_analysis.hpp>
#include <iostream>
using namespace std;
void 	CarStateAnalysis::UpdateCar(const CarInfo &carInfo)
{
	if (!b_available)
		b_available = true;

    cout << b_init << endl;
	if (b_init)
	{
		carState.is_position_left = (carInfo.Y < min_car_y);
		carState.is_position_right = (carInfo.Y > max_car_y);
		carState.is_position_front = (carInfo.X > max_car_x);
		carState.is_position_back = (carInfo.X < min_car_x);
		carState.is_yaw_left = (carInfo.yaw > max_car_yaw);
		carState.is_yaw_right = (carInfo.yaw < min_car_yaw);
		carState.is_steering_left = (carInfo.steering > max_front_steering_angle);
		carState.is_steering_right = (carInfo.steering < min_front_steering_angle);
		carState.is_wheelbase_short = (carInfo.length_wheelbase < min_wheelbase);
		carState.is_wheelbase_long = (carInfo.length_wheelbase > max_wheelbase);
		carState.is_wheel_distance_short = (carInfo.length_wheel_distance < min_wheels_distance);
		carState.is_wheel_distance_long = (carInfo.length_wheel_distance > max_wheels_distance);

        cout << carInfo.X << " " << carInfo.Y << " " << carInfo.yaw << " " << carInfo.steering << " " << carInfo.length_wheelbase << " " << carInfo.length_wheel_distance << endl;
        cout << min_car_y << " " << min_car_x << " " << min_car_yaw << " " << min_front_steering_angle << " " << min_wheelbase << " " << min_wheels_distance << endl;
	}
}

bool 	CarStateAnalysis::GetCarState(CarState &carState)
{
	if (b_available && b_init)
	{
		carState = this->carState;
		return true;
	}
	else
		return false;
}